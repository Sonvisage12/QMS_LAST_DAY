#include <SPI.h>
#include <MFRC522.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include "PatientRFIDMappings.h"
#include <esp_now.h>
#include <WiFi.h>
#include <map>
#include <vector>
#include <algorithm>
#include "SharedQueue.h"
#include <Wire.h>
#include "RTClib.h"
#define IR_SENSOR_PIN 16  // GPIO 0
#include <ESP32Servo.h>
#define SERVO_PIN 26
Servo myServo;
bool isOpen = false;
bool arbiter = true;
#define RST_PIN  5
#define SS_PIN   4
#define GREEN_LED_PIN 15
#define RED_LED_PIN   2
//#define BLUE_LED_PIN   
#define BUZZER   27
MFRC522 mfrc522(SS_PIN, RST_PIN);
Preferences prefs;
int k;
#define MAX_RETRIES 5
#define RETRY_INTERVAL 300  // ms
#define MESSAGE_TIMEOUT 10000 // 10 seconds
bool waitingForCard = false;
SharedQueue sharedQueue("rfid-patients");
SharedQueue sharedQueueA("queue-A");
SharedQueue sharedQueueB("queue-B");
SharedQueue sharedQueueC("queue-C");

typedef struct {
    uint32_t id;
    uint8_t targetMac[6];
    QueueItem item;
    uint8_t retries;
    unsigned long lastAttempt;
    bool requiresAck;
} PendingMessage;
std::vector<PendingMessage> pendingMessages;
uint32_t messageCounter = 0;

bool isMaster = false;          // True if this node is the current master
int myIndex = 0;               // This node's index in arrivalMACs
int currentMasterIndex = 0;     // Index of the currently known master
//const int numDoctorNodes = sizeof(doctorMACs) / sizeof(doctorMACs[0]);
RTC_DS3231 rtc;
bool isArrivalNode = false, isDoctorNode = false;
std::map<String, unsigned long> recentUIDs;
const unsigned long UID_CACHE_TTL_MS = 2000;
const uint8_t arrivalMACs[][6] = {
   {0x78, 0x42, 0x1C, 0x6C, 0xE4, 0x9C},
   {0x5C, 0x01, 0x3B, 0x98, 0xDB, 0x04},
    {0x5C, 0x01, 0x3B, 0x97, 0x54, 0xB4},
    {0x78, 0x1C, 0x3C, 0x2D, 0xA2, 0xA4},
    {0x00, 0x4B, 0x12, 0x97, 0x2E, 0xA4}, //78:1C:3C:2D:A2:A4
    {0x78, 0x1C, 0x3C, 0xE6, 0x6C, 0xB8}, //78:1C:3C:E6:6C:B8
    {0x78, 0x1C, 0x3C, 0xE3, 0xAB, 0x30}, //78:1C:3C:E3:AB:30
    //5C:01:3B:98:DB:04
     //78:42:1C:6C:E4:9C
};
const int numArrivalNodes = sizeof(arrivalMACs) / sizeof(arrivalMACs[0]);

const uint8_t doctorMACs[][6] = {
    {0x78, 0x42, 0x1C, 0x6C, 0xA8, 0x3C},
    {0x5C, 0x01, 0x3B, 0x98, 0x3C, 0xEC},
    {0x5C, 0x01, 0x3B, 0x98, 0xE8, 0x2C},//5C:01:3B:98:E8:2C
    {0x78, 0x1C, 0x3C, 0xE5, 0x50, 0x0C}
};

const int numDoctorNodes = sizeof(doctorMACs) / sizeof(doctorMACs[0]);  // 👈 Must be AFTER doctorMACs is declared

bool attemptSend(PendingMessage &msg) {
    esp_err_t result = esp_now_send(msg.targetMac, (uint8_t*)&msg.item, sizeof(msg.item));
    
    if (result == ESP_OK) {
        msg.lastAttempt = millis();
        msg.retries++;
        return true;
    }
    return false;
}

void manageRetries() {
    unsigned long now = millis();
    
    for (auto it = pendingMessages.begin(); it != pendingMessages.end(); ) {
        // Check for timeout
        if (now - it->lastAttempt > MESSAGE_TIMEOUT) {
            Serial.printf("Message %d timed out after %d retries\n", it->id, it->retries);
            it = pendingMessages.erase(it);
            continue;
        }
        
        // Check if ready for retry
        if (now - it->lastAttempt >= RETRY_INTERVAL && it->retries < MAX_RETRIES) {
            if (attemptSend(*it)) {
                Serial.printf("Retry %d for message %d\n", it->retries, it->id);
            }
        }
        
        ++it;
    }
}

bool sendMessageWithGuarantee(const uint8_t* targetMac, const QueueItem &item, bool ackRequired = true) {
    PendingMessage msg;
    msg.id = messageCounter++;
    memcpy(msg.targetMac, targetMac, 6);
    msg.item = item;
    msg.retries = 0;
    msg.lastAttempt = millis();
    msg.requiresAck = ackRequired;
    
    pendingMessages.push_back(msg);
    return attemptSend(msg);
}

void broadcastToArrivalNodes(const QueueItem &item) {
String myMAC = WiFi.macAddress();
if (myMAC == String(item.sourceMAC)) {
    Serial.println("🔁 Received own broadcast. Skipping rebroadcast.");
    return;
    for (int i = 0; i < numArrivalNodes; i++) {
      if (i == myIndex) continue; // Don't send to self
       // esp_now_send(arrivalMACs[i], (uint8_t*)&item, sizeof(item));
 sendMessageWithGuarantee(arrivalMACs[i], item);
    }
    }
}

void notifyDoctorsOfNewMaster() {
    if (myIndex < 0 || myIndex >= numArrivalNodes) {
        Serial.printf("❌ Cannot notify doctors: invalid myIndex = %d\n", myIndex);
        return;
    }

    QueueItem announce = {};
    announce.number = 0; // Optional, set to 0 or omit if not needed
    announce.addToQueue = false;
    announce.removeFromQueue = false;
    strcpy(announce.uid, "MASTER_CHANGE");
    strcpy(announce.type, "MASTER");

    uint8_t myMAC[6];
    WiFi.macAddress(myMAC);
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             myMAC[0], myMAC[1], myMAC[2], myMAC[3], myMAC[4], myMAC[5]);
    strncpy(announce.sourceMAC, macStr, sizeof(announce.sourceMAC));

    for (int i = 0; i < numDoctorNodes; ++i) {
        announce.node = i + 1;
       // esp_now_send(doctorMACs[i], (uint8_t*)&announce, sizeof(announce));
        sendMessageWithGuarantee(doctorMACs[i], announce);
    }

    Serial.printf("📢 New master announced (MAC: %s) to all doctors.\n", announce.sourceMAC);
}


void sendQueueToAllArrivalNodes(SharedQueue& queue, const char* queueName) {
   QueueItem item;
    std::vector<QueueEntry> entries = queue.getAll();
  String myMAC = WiFi.macAddress();
if (myMAC == String(item.sourceMAC)) {
    Serial.println("🔁 Received own broadcast. Skipping rebroadcast.");
    return;
    //std::vector<QueueEntry> entries = queue.getAll();
    for (const auto& entry : entries) {
       // QueueItem item;
        strncpy(item.uid, entry.uid.c_str(), sizeof(item.uid));
        strncpy(item.timestamp, entry.timestamp.c_str(), sizeof(item.timestamp));
        item.number = entry.number;
        item.node = 0; // Not tied to a specific doctor node
        item.addToQueue = true;
        item.removeFromQueue = false;
        strncpy(item.queueID, queueName, sizeof(item.queueID));

        broadcastToArrivalNodes(item);}
    }
}
void handleAddToQueueAsMaster(String uid, String timestamp, int number, String queueID) {
    if (queueID == "queueA") {
        if (sharedQueue.exists(uid)) {
            sharedQueue.removeByUID(uid);
            sharedQueueA.add(uid, timestamp, number);
        }
    } else if (queueID == "queueB") {
        if (sharedQueueA.exists(uid)) {
            sharedQueueA.removeByUID(uid);
            sharedQueueB.add(uid, timestamp, number);
        }
    } else if (queueID == "mix") {
        if (!sharedQueueC.exists(uid)) {
            sharedQueueC.add(uid, timestamp, number);
        }
    } else {
        if (!sharedQueue.exists(uid) && !sharedQueueA.exists(uid) && !sharedQueueB.exists(uid)) {
            sharedQueue.add(uid, timestamp, number);
        }
    }

    createMixedQueue();         // only master runs this
    broadcastAllQueues();       // master updates others
    printAllQueues();
}

void validateQueues() {
    auto checkQueue = [](SharedQueue& queue, const char* name) {
        for (const auto& entry : queue.getAll()) {
            if (!isValidUID(entry.uid.c_str())) {
                Serial.printf("❌ Corrupted UID in %s: %s\n", name, entry.uid.c_str());
                queue.removeByUID(entry.uid);
            }
        }
    };

    checkQueue(sharedQueue, "sharedQueue");
    checkQueue(sharedQueueA, "sharedQueueA");
    checkQueue(sharedQueueB, "sharedQueueB");
    checkQueue(sharedQueueC, "sharedQueueC");
}
void handleQueuePlacement(String uid, int number) {
    // Get current local time from RTC
    DateTime now = rtc.now();
    char timeBuffer[25];
    snprintf(timeBuffer, sizeof(timeBuffer), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(), 
             now.hour(), now.minute(), now.second());
    String timestamp = String(timeBuffer);

    // Prepare queue update message
    QueueItem update;
    strncpy(update.uid, uid.c_str(), sizeof(update.uid));
    strncpy(update.timestamp, timestamp.c_str(), sizeof(update.timestamp));
    update.number = number;
    update.addToQueue = true;
    update.removeFromQueue = false;
    
    // Include node's MAC address
    uint8_t mac[6];
    WiFi.macAddress(mac);
    snprintf(update.sourceMAC, sizeof(update.sourceMAC), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    if (sharedQueueA.exists(uid)) {
        // Move from Queue A to Queue B with current time
        sharedQueueA.removeByUID(uid);
        sharedQueueB.add(uid, timestamp, number);
        strncpy(update.queueID, "queueB", sizeof(update.queueID));
        
        Serial.println("🔄 UID moved from SharedQueueA to SharedQueueB.");
        broadcastToArrivalNodes(update);
        
    } else if (sharedQueue.exists(uid) || sharedQueueB.exists(uid)) {
        // Patient already in a queue
        Serial.println("⚠️ UID already in a queue. Skipping addition.");
        
    } else {
        // New patient - add to main queue with current time
        sharedQueue.add(uid, timestamp, number);
        strncpy(update.queueID, "main", sizeof(update.queueID));
        
        Serial.println("✅ UID added to SharedQueue.");
        broadcastToArrivalNodes(update);
    }

    // Update mixed queue if this is the arbiter
    if(arbiter) {
        createMixedQueue();
        broadcastAllQueues();
    }

    // Print all queues
    sharedQueue.print(); 
    sharedQueueA.print(); 
    sharedQueueB.print(); 
    sharedQueueC.print();
}

void handleQueuePlacement1(String uid, int number, String queueID) {
    // Get current local time from RTC
    DateTime now = rtc.now();
    char timeBuffer[25];
    snprintf(timeBuffer, sizeof(timeBuffer), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(), 
             now.hour(), now.minute(), now.second());
    String timestamp = String(timeBuffer);

    // Prepare queue update message
    QueueItem update;
    strncpy(update.uid, uid.c_str(), sizeof(update.uid));
    strncpy(update.timestamp, timestamp.c_str(), sizeof(update.timestamp));
    update.number = number;
    update.addToQueue = true;
    update.removeFromQueue = false;
    strncpy(update.queueID, queueID.c_str(), sizeof(update.queueID));
    
    // Include node's MAC address
    uint8_t mac[6];
    WiFi.macAddress(mac);
    snprintf(update.sourceMAC, sizeof(update.sourceMAC), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
 if (queueID != "queueA" && queueID != "queueB" && 
        (sharedQueue.exists(uid) || sharedQueueA.exists(uid) || sharedQueueB.exists(uid))) {
        Serial.println("⚠️ UID already in another queue. Ignoring.");
        return;
    }
    if (queueID == "queueA") {
        // Move from SharedQueue to A (doctor call)
        if (sharedQueue.exists(uid)) {
            sharedQueue.removeByUID(uid);
            sharedQueueA.add(uid, timestamp, number);
            Serial.println("🟡 UID moved from SharedQueue to SharedQueueA.");
            
            // Broadcast update to other nodes
            broadcastToArrivalNodes(update);
        } else {
            Serial.println("⚠️ UID not in SharedQueue; cannot move to A.");
        }

    } else if (queueID == "queueB") {
        // Move from A to B (patient re-registers)
        if (sharedQueueA.exists(uid)) {
            sharedQueueA.removeByUID(uid);
            sharedQueueB.add(uid, timestamp, number);
            Serial.println("🔵 UID moved from SharedQueueA to SharedQueueB.");
            
            // Broadcast update to other nodes
            broadcastToArrivalNodes(update);
        } else {
            Serial.println("⚠️ UID not in SharedQueueA; cannot move to B.");
        }

    } else if (queueID == "mix") {
        // Add to mixed display queue if not present
        if (!sharedQueueC.exists(uid)) {
            sharedQueueC.add(uid, timestamp, number);
            Serial.println("🟢 UID added to SharedQueueC (mixed queue).");
            
            // Broadcast update to other nodes
            broadcastToArrivalNodes(update);
        }

    } else {
        // Default: add to SharedQueue if not in other queues
        if (!sharedQueue.exists(uid) && !sharedQueueA.exists(uid) && !sharedQueueB.exists(uid)) {
            sharedQueue.add(uid, timestamp, number);
            Serial.println("✅ UID added to SharedQueue (initial).");
            
            // Broadcast update to other nodes
            broadcastToArrivalNodes(update);
        } else {
            Serial.println("⚠️ UID already in another queue; not added to SharedQueue.");
        }
    }

    // Update mixed queue if this is the arbiter
    if(arbiter && (queueID == "queueA" || queueID == "queueB" || queueID == "mix")) {
        createMixedQueue();
        broadcastAllQueues();
    }

    printAllQueues();
}
void cleanupRecentUIDs() {
    unsigned long now = millis();
    for (auto it = recentUIDs.begin(); it != recentUIDs.end(); ) {
        if (now - it->second > UID_CACHE_TTL_MS)
            it = recentUIDs.erase(it);
        else
            ++it;
    }
}
bool isValidUID(const char* uid) {
    if (strlen(uid) != 14) return false; // Expected length for your UIDs
    for (int i = 0; i < 14; i++) {
        if (!isxdigit(uid[i])) return false;
    }
    return true;
}

// Usage in processCard():

void sendTimeToAllArrivalNodes() {
    if (!isMaster) {
        Serial.println("⚠️ Not master - cannot send time sync");
        return;
    }

    DateTime now = rtc.now();
    char timeBuffer[25];
    snprintf(timeBuffer, sizeof(timeBuffer), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());

    QueueItem timeSync;
    strcpy(timeSync.uid, "RTC_SYNC");
    strncpy(timeSync.timestamp, timeBuffer, sizeof(timeSync.timestamp));
    timeSync.addToQueue = false;
    timeSync.removeFromQueue = false;
    
    // Include master's MAC address
    uint8_t myMAC[6];
    WiFi.macAddress(myMAC);
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             myMAC[0], myMAC[1], myMAC[2], myMAC[3], myMAC[4], myMAC[5]);
    strncpy(timeSync.sourceMAC, macStr, sizeof(timeSync.sourceMAC));

    for (int i = 0; i < numArrivalNodes; i++) {
        if (i == myIndex) continue; // Don't send to self
        esp_err_t result = esp_now_send(arrivalMACs[i], (uint8_t*)&timeSync, sizeof(timeSync));
        if (result == ESP_OK) {
            Serial.printf("🕒 Time sync sent to node %d\n", i);
        } else {
            Serial.printf("❌ Failed to send time sync to node %d\n", i);
        }
    }

    Serial.printf("🕒 Master broadcasted RTC time: %s\n", timeBuffer);
}

void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
    if (len != sizeof(QueueItem)) {
        Serial.println("⚠️ Invalid message length. Ignoring.");
        return;
    }
    QueueItem item;
    memcpy(&item, incomingData, sizeof(item));
    String uidStr(item.uid);
      if (!isValidUID(item.uid)) {
        Serial.println("❌ Corrupted UID in received message. Ignoring.");
        return;
    }
    if (sharedQueue.exists(uidStr) || sharedQueueA.exists(uidStr) || sharedQueueB.exists(uidStr)) {
    Serial.println("⏳ Duplicate UID received. Ignoring.");
    return;
    }
  if (isUIDInAnyQueue(item.uid)) {
    Serial.println("⚠️ UID already in a queue. Ignoring.");
    return;
}
  
    // Identify source role
    bool fromArrival = false, fromDoctor = false;
    int srcIndex = -1, doctorNodeID = -1;
    const uint8_t* mac = recvInfo->src_addr;
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", 
             mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    // Check if source is a known arrival node
    for (int i = 0; i < numArrivalNodes; ++i) {
        if (memcmp(mac, arrivalMACs[i], 6) == 0) { fromArrival = true; srcIndex = i; break; }
    }
    // Check if source is a known doctor node
    for (int i = 0; i < numDoctorNodes; ++i) {
        if (memcmp(mac, doctorMACs[i], 6) == 0) { fromDoctor = true; doctorNodeID = i+1; break; }
    }

    if (fromArrival) {
        Serial.printf("📩 Arrival message from node %d (%s)\n", srcIndex, macStr);
        cleanupRecentUIDs();
        if (strcmp(item.uid, "RTC_SYNC") == 0) {

          if (!isMaster && srcIndex == currentMasterIndex) {
    static unsigned long lastRTCUpdate = 0;
    static bool rtcSetOnce = false;

    unsigned long nowMillis = millis();
    if (!rtcSetOnce || nowMillis - lastRTCUpdate >= 7200000UL) {
        DateTime newTime = parseDateTime(String(item.timestamp));
        rtc.adjust(newTime);
        lastRTCUpdate = nowMillis;
        rtcSetOnce = true;
        Serial.printf("🕒 RTC updated from master (%d) to: %s\n", currentMasterIndex, item.timestamp);
    } else {
        Serial.println("⏳ Ignoring RTC sync: less than 2 hours since last update.");
    }
} else {
    Serial.printf("⚠️ Ignoring RTC_SYNC: not from current master (%d), came from %d\n", currentMasterIndex, srcIndex);
}

if (len == sizeof(AckMessage)) {
        AckMessage* ack = (AckMessage*)incomingData;
        auto it = std::find_if(pendingMessages.begin(), pendingMessages.end(),
            [ack](const PendingMessage& m) { return m.id == ack->messageId; });
        
        if (it != pendingMessages.end()) {
            if (ack->success) {
                pendingMessages.erase(it);
            } else {
                it->lastAttempt = millis() - RETRY_INTERVAL; // Force retry
            }
        }
        return;
    }


              if (!isMaster) {
                  // Parse timestamp from master
                  DateTime newTime = parseDateTime(String(item.timestamp));
                  rtc.adjust(newTime);
                  Serial.printf("🕒 RTC updated from master to: %s\n", item.timestamp);
              }
              return;
            }

        // Handle add-to-queue events
        if (item.addToQueue && !item.removeFromQueue) {
            if (recentUIDs.count("add:" + uidStr)) {
                Serial.println("⏳ Duplicate add event, ignoring.");
                return;
            }
            recentUIDs["add:" + uidStr] = millis();
            // Check for master change
            if (!isMaster) {
                if (srcIndex != currentMasterIndex) {
                    // Not from current master -> potential master change or failover
                    if (srcIndex > myIndex) {
                        // A node with higher index sent this -> promote self to master
                        Serial.println("⚠️ Master down. Promoting self to master.");
                        isMaster = true;
                        currentMasterIndex = myIndex;
                        notifyDoctorsOfNewMaster();
                        //notifyDoctorsOfNewMaster();
                        sendTimeToAllArrivalNodes();
                    } else if (srcIndex != myIndex) {
                        // A lower-index node (higher priority) sent this -> it is new master
                        Serial.printf("🔁 Updating current master to node %d.\n", srcIndex);
                        currentMasterIndex = srcIndex;
                    }
                }
            }
            // Now process the add event
            if (isMaster) {
                // Master mode: handle queue placement fully and broadcast to others
                handleAddToQueueAsMaster(String(item.uid), String(item.timestamp), item.number, String(item.queueID));
            } else {
                // Secondary mode: update local queue state (no rebroadcast)
                handleQueuePlacement1(uidStr, item.number, String(item.queueID));
            }
        }
        // Handle remove-from-queue events
        else if (!item.addToQueue && item.removeFromQueue) {
            if (recentUIDs.count("rem:" + uidStr)) {
                Serial.println("⏳ Duplicate remove event, ignoring.");
                return;
            }
            recentUIDs["rem:" + uidStr] = millis();
            if (!isMaster) {
                if (srcIndex != currentMasterIndex) {
                    // Source is not the known master -> update master (likely a new master broadcasting removal)
                    if (srcIndex != myIndex) {
                        Serial.printf("🔁 Master updated to node %d (for removal event).\n", srcIndex);
                        currentMasterIndex = srcIndex;
                    }
                    // (If srcIndex > myIndex, theoretically could mean a higher-index tried removal, 
                    // but normal nodes don't send removal events, so we ignore that case.)
                }
            }
            // Update local queues for removal
            if (sharedQueue.exists(uidStr)) {
                sharedQueue.removeByUID(uidStr);
                sharedQueueC.removeByUID(uidStr);
                sharedQueueA.add(uidStr, String(item.timestamp), item.number);
                Serial.printf("🗑️ %s removed from main queue, added to QueueA.\n", item.uid);
            } else {
                // If not in main, remove from secondary queues B/C
                sharedQueueC.removeByUID(uidStr);
                sharedQueueB.removeByUID(uidStr);
                Serial.printf("🗑️ %s removed from QueueC/QueueB (if present).\n", item.uid);
            }
        }
        printAllQueues();

         if (item.requiresAck) {  // Changed from msg.requiresAck
        AckMessage ack;
        ack.messageId = item.messageID;
        ack.success = true;
        esp_now_send(recvInfo->src_addr, (uint8_t*)&ack, sizeof(ack));
    }
        return;  // done handling arrival message
    }

    if (fromDoctor && doctorNodeID != -1) {
        Serial.printf("📩 Doctor message from Node %d (%s)\n", doctorNodeID, macStr);
        if (strcmp(item.uid, "REQ_NEXT") == 0) {
            Serial.println("👨‍⚕️ Doctor requests next patient.");
            if (!isMaster) {
                // No response from true master, take over
                Serial.println("⚠️ Current master unresponsive. Becoming master to serve doctor.");
                isMaster = true;
                currentMasterIndex = myIndex;
            }
            if (!sharedQueueC.empty()) {
                // Prepare next patient info
                QueueEntry entry = sharedQueueC.front();
                // Determine origin queue and rotate it
               if (sharedQueue.exists(entry.uid)) {
                    sharedQueue.pop();
                    sharedQueue.insertAt(10, entry.uid, entry.timestamp, entry.number); // insert into main queue
                } else if (sharedQueueB.exists(entry.uid)) {
                    sharedQueueB.pop();
                    sharedQueueB.insertAt(10, entry.uid, entry.timestamp, entry.number); // insert into queue B
                }

                // Remove from front of mixed queue and re-add to back (to maintain ordering for round-robin)
                sharedQueueC.pop();
              //  sharedQueueC.push(entry);
               sharedQueueC.insertAt(10, entry.uid, entry.timestamp, entry.number); 
                // Send patient info to requesting doctor
                QueueItem response{};
                strncpy(response.uid, entry.uid.c_str(), sizeof(response.uid));
                strncpy(response.timestamp, entry.timestamp.c_str(), sizeof(response.timestamp));
                response.number = entry.number;
                response.node   = doctorNodeID;
                response.addToQueue = false;
                response.removeFromQueue = false;
                esp_now_send(mac, (uint8_t*)&response, sizeof(response));  // send to this doctor
                Serial.printf("✅ Sent next patient %s to Doctor %d.\n", response.uid, doctorNodeID);
                // Broadcast an update to arrival nodes about this patient's new position (rotated to queueA or back of queue)
                QueueItem update = response;
                update.addToQueue = true;
                update.removeFromQueue = false;
                if (sharedQueue.exists(entry.uid)) {
                    strncpy(update.queueID, "main", sizeof(update.queueID));
                } else {
                    strncpy(update.queueID, "queueB", sizeof(update.queueID));
                }
                broadcastToArrivalNodes(update);
                Serial.println("🔄 Broadcasted queue update to all arrival nodes (post-doctor call).");
            } else {
                // No patient waiting
                QueueItem noPat{};
                strncpy(noPat.uid, "NO_PATIENT", sizeof(noPat.uid));
                noPat.number = 0;
                noPat.node   = doctorNodeID;
                noPat.addToQueue = false;
                noPat.removeFromQueue = false;
                esp_now_send(mac, (uint8_t*)&noPat, sizeof(noPat));
                Serial.println("⚠️ No patients in queue. Informed doctor.");
            }
        } else if (item.removeFromQueue) {
            // Doctor signals patient has been handled/removed
            Serial.println("👨‍⚕️ Doctor signals removal of patient from queue.");
            if (!isMaster) {
                // If this node wasn't master, assume master failed to handle this and take over
                Serial.println("⚠️ Master not responding to removal. Becoming master to handle removal.");
                isMaster = true;
                currentMasterIndex = myIndex;
            }
            // Broadcast removal command to all other arrival nodes
            item.addToQueue = false;
            item.removeFromQueue = true;
            strncpy(item.queueID, "", sizeof(item.queueID));  // not strictly needed to set
            for (int i = 0; i < numArrivalNodes; ++i) {
                if (i == myIndex) continue;
                esp_now_send(arrivalMACs[i], (uint8_t*)&item, sizeof(item));
            }
            Serial.println("📢 Broadcasted removal to all nodes.");
            // Update local queues for removal (same as above)
            if (sharedQueue.exists(uidStr)) {
                sharedQueue.removeByUID(uidStr);
                sharedQueueC.removeByUID(uidStr);
                sharedQueueA.add(uidStr, String(item.timestamp), item.number);
                Serial.printf("🗑️ %s removed from main queue, added to QueueA.\n", item.uid);
            } else {
                sharedQueueC.removeByUID(uidStr);
                sharedQueueB.removeByUID(uidStr);
                Serial.printf("🗑️ %s removed from QueueC/QueueB.\n", item.uid);
            }
            // If no one left waiting, inform all doctors to stop calling
            if (sharedQueueC.empty()) {
                QueueItem noPat{};
                strncpy(noPat.uid, "NO_PATIENT", sizeof(noPat.uid));
                noPat.number = 0;
                noPat.addToQueue = false;
                noPat.removeFromQueue = false;
                for (int i = 0; i < numDoctorNodes; ++i) {
                    noPat.node = i+1;
                    esp_now_send(doctorMACs[i], (uint8_t*)&noPat, sizeof(noPat));
                }
                Serial.println("⚠️ Queue empty. Notified all doctors with NO_PATIENT.");
            }
        }
        else if (strcmp(item.uid, "CLEAR_QUEUE") == 0) {
    Serial.println("🧹 Doctor requested CLEAR QUEUE.");
    clearAllQueues();

    // If master, optionally re-broadcast to all other arrival nodes
    if (isMaster) {
        for (int i = 0; i < numArrivalNodes; ++i) {
            if (i == myIndex) continue;
            esp_now_send(arrivalMACs[i], (uint8_t*)&item, sizeof(item));
        }
        
        Serial.println("📢 CLEAR_QUEUE broadcasted to all arrival nodes.");
    }
    // Send "NO_PATIENT" (patient number 0) to all doctor nodes
QueueItem zeroPatient{};
strncpy(zeroPatient.uid, "NO_PATIENT", sizeof(zeroPatient.uid));
zeroPatient.number = 0;
zeroPatient.addToQueue = false;
zeroPatient.removeFromQueue = false;

for (int i = 0; i < numDoctorNodes; ++i) {
    zeroPatient.node = i + 1;
    esp_now_send(doctorMACs[i], (uint8_t*)&zeroPatient, sizeof(zeroPatient));
}
Serial.println("📤 Sent NO_PATIENT (0) to all doctors after CLEAR_QUEUE.");

  //printAllQueues();
    return;
}
validateQueues();
        printAllQueues();
        return;
    }
}

int getPermanentNumber(String uid) {
    if (!LittleFS.begin()) {
        Serial.println("❌ LittleFS mount failed");
        return -1;
    }

    File file = LittleFS.open("/rfid_mappings.txt", "r");
    if (!file) {
        Serial.println("❌ File not found");
        return -1;
    }

    int pid = -1; // Default: Not found

    while (file.available()) {
        String line = file.readStringUntil('\n');
        int separatorIndex = line.indexOf('=');
        
        if (separatorIndex != -1) {
            String storedUID = line.substring(0, separatorIndex);
            if (storedUID == uid) {
                pid = line.substring(separatorIndex + 1).toInt();
                break;
            }
        }
    }

    file.close();
    return pid;
}




void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.println("Delivery succeeded");
    } else {
        Serial.println("Delivery failed - will retry");
        // The retry mechanism in loop() will handle resending
    }
}

bool isTimeToClearQueues() {
    DateTime now = rtc.now();
    return (now.hour() == 4 && now.minute() == 00 && now.second() == 0);
}

bool alreadyClearedToday() {
    static int lastClearedDay = -1;
    DateTime now = rtc.now();
    
    if (lastClearedDay != now.day()) {
        lastClearedDay = now.day();
        return false;
    }
    return true;
}
void processCard(String uid) {
    // Check if UID already exists in ANY queue
   if (!isValidUID(uid.c_str())) {
    Serial.println("❌ Invalid UID format. Ignoring.");
    return;
}

    DateTime now = rtc.now();
    char timeBuffer[25];
    snprintf(timeBuffer, sizeof(timeBuffer), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    String timeStr = String(timeBuffer);

    int pid = getPermanentNumber(uid);
    if (pid == -1) pid = 0;

    QueueItem item;
    strncpy(item.uid, uid.c_str(), sizeof(item.uid));
    strncpy(item.timestamp, timeStr.c_str(), sizeof(item.timestamp));
    item.number = pid;
    item.addToQueue = true;
    item.removeFromQueue = false;

     if (sharedQueue.exists(uid) || sharedQueueA.exists(uid) || sharedQueueB.exists(uid)) {
        Serial.println("⚠️ UID already in queue. Ignoring.");
        blinkLED(RED_LED_PIN);
        return;  // Skip processing if already exists
    }
    
if (isUIDInAnyQueue(uid)) {
    Serial.println("⚠️ UID already in a queue. Ignoring.");
    return;
}
    // Add to main queue (since it's a new UID)
    sharedQueue.add(uid, timeStr, pid);
    strncpy(item.queueID, "main", sizeof(item.queueID));

    // Broadcast to all arrival nodes
    for (int i = 0; i < numArrivalNodes; i++) {
        esp_now_send(arrivalMACs[i], (uint8_t*)&item, sizeof(item));
    }

    Serial.println("✅ UID added to SharedQueue.");
    blinkLED(GREEN_LED_PIN); validateQueues();
    printAllQueues();
}

// void processCard(String uid) {
//     DateTime now = rtc.now();
//     char timeBuffer[25];
//     snprintf(timeBuffer, sizeof(timeBuffer), "%04d-%02d-%02d %02d:%02d:%02d",
//              now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
//     String timeStr = String(timeBuffer);

//     int pid = getPermanentNumber(uid);
//     if (pid == -1) pid = 0;

//     QueueItem item;
//     strncpy(item.uid, uid.c_str(), sizeof(item.uid));
//     strncpy(item.timestamp, timeStr.c_str(), sizeof(item.timestamp));
//     item.number = pid;
//     item.addToQueue = true; item.removeFromQueue = false;

//     if (sharedQueueA.exists(uid)) {
//         sharedQueueA.removeByUID(uid);
//         sharedQueueB.add(uid, timeStr, pid);
//         strncpy(item.queueID, "queueB", sizeof(item.queueID));
// // Tag the origin
//         for (int i = 0; i < numArrivalNodes; i++) esp_now_send(arrivalMACs[i], (uint8_t*)&item, sizeof(item));
//         Serial.println("🔄 UID moved from SharedQueueA to SharedQueueB.");
//         if(k==1){ blinkLED(GREEN_LED_PIN);}
//          // Success indicator digitalWrite(GREEN_LED_PIN, LOW );
//     } else if (sharedQueue.exists(uid) || sharedQueueB.exists(uid)) {
//        //strncpy(item.queueID, queueName, sizeof(item.queueID));  // Tag the origin
//         Serial.println("⚠️ UID already in queue.");
//          if(k==1){ blinkLED(RED_LED_PIN); }
//          // Warning indicator
//     } else {
//         sharedQueue.add(uid, timeStr, pid);
//          strncpy(item.queueID, "main", sizeof(item.queueID));
//         for (int i = 0; i < numArrivalNodes; i++) esp_now_send(arrivalMACs[i], (uint8_t*)&item, sizeof(item));
//         Serial.println("✅ UID added to SharedQueue.");
//          if(k==1){blinkLED(GREEN_LED_PIN);}
//           // Success indicator
//     }
//     printAllQueues();
// }
// void processCard1(String uid) {
//     DateTime now = rtc.now();
//     char timeBuffer[25];
//     snprintf(timeBuffer, sizeof(timeBuffer), "%04d-%02d-%02d %02d:%02d:%02d",
//              now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
//     String timeStr = String(timeBuffer);

//     int pid = getPermanentNumber(uid);
//     if (pid == -1) pid = 0;

//     QueueItem item;
//     strncpy(item.uid, uid.c_str(), sizeof(item.uid));
//     strncpy(item.timestamp, timeStr.c_str(), sizeof(item.timestamp));
//     item.number = pid;
//     item.addToQueue = true; item.removeFromQueue = false;

//     if (sharedQueueA.exists(uid)) {
        
//        esp_now_send(arrivalMACs[0], (uint8_t*)&item, sizeof(item));
//         Serial.println("🔄 UID moved from SharedQueueA to SharedQueueB.");
//         blinkLED(GREEN_LED_PIN);  // Success indicator digitalWrite(GREEN_LED_PIN, LOW );
//     } else if (sharedQueue.exists(uid) || sharedQueueB.exists(uid)) {
//         Serial.println("⚠️ UID already in queue.");
//         blinkLED(RED_LED_PIN);  // Warning indicator
//     } else {
//         //esp_now_send(arrivalMACs[0], (uint8_t*)&item, sizeof(item));
//          for (int i = 0; i < numArrivalNodes; i++) esp_now_send(arrivalMACs[i], (uint8_t*)&item, sizeof(item));
//         Serial.println("✅ UID added to SharedQueue.");
//         blinkLED(GREEN_LED_PIN);  // Success indicator
//     }
//     printAllQueues();
// }
void broadcastAllQueues() {
    auto broadcastQueue = [](SharedQueue& queue, const char* queueName) {
        std::vector<QueueEntry> entries = queue.getAll();
        for (const auto& entry : entries) {
            QueueItem item;
            strncpy(item.uid, entry.uid.c_str(), sizeof(item.uid));
            strncpy(item.timestamp, entry.timestamp.c_str(), sizeof(item.timestamp));
            item.number = entry.number;
            item.node = 0;
            item.addToQueue = true;
            item.removeFromQueue = false;
            strncpy(item.queueID, queueName, sizeof(item.queueID));  // Tag the origin
            broadcastToArrivalNodes(item);
        }
    };

    broadcastQueue(sharedQueue, "main");
    broadcastQueue(sharedQueueA, "queueA");
    broadcastQueue(sharedQueueB, "queueB");
    broadcastQueue(sharedQueueC, "mix");

    Serial.println("📡 Broadcasted all queues to arrival nodes with queue IDs.");
}

//Only arbiter should contain the following function createMixedQueue() 

// void createMixedQueue() {
//     sharedQueueC.clear();

//     std::vector<QueueEntry> entriesA = sharedQueue.getAll();
//     std::vector<QueueEntry> entriesB = sharedQueueB.getAll();
//     size_t indexA = 0, indexB = 0;

//     while (indexA < entriesA.size() || indexB < entriesB.size()) {
//         if (indexB < entriesB.size()) {
//             for (int i = 0; i < 10 && indexB < entriesB.size(); i++, indexB++) {
//                 sharedQueueC.add1(entriesB[indexB].uid, entriesB[indexB].timestamp, entriesB[indexB].number);
//             }
//         }

//         if (indexA < entriesA.size()) {
//             for (int i = 0; i < 5 && indexA < entriesA.size(); i++, indexA++) {
//                 sharedQueueC.add1(entriesA[indexA].uid, entriesA[indexA].timestamp, entriesA[indexA].number);
//             }
//         }
//     }

//     // Broadcast updated sharedQueueC entries to all Arrival Nodes
//     std::vector<QueueEntry> entriesC = sharedQueueC.getAll();
//     for (const auto& entry : entriesC) {
//         QueueItem item;
//         strncpy(item.uid, entry.uid.c_str(), sizeof(item.uid));
//         strncpy(item.timestamp, entry.timestamp.c_str(), sizeof(item.timestamp));
//         strncpy(item.queueID, "mix", sizeof(item.queueID));
//         item.number = entry.number;
//         item.addToQueue = true;
//         item.removeFromQueue = false;
//         broadcastToArrivalNodes(item);

//     }
//     Serial.println("📡 Broadcasted sharedQueueC to all Arrival Nodes.");
// }
bool isUIDInAnyQueue(String uid) {
    return (
        sharedQueue.exists(uid) ||
        sharedQueueA.exists(uid) ||
        sharedQueueB.exists(uid) ||
        sharedQueueC.exists(uid)
    );
}
void createMixedQueue() {
    sharedQueueC.clear(); // Clear before rebuilding

    // Add from Queue B (higher priority)
    std::vector<QueueEntry> entriesB = sharedQueueB.getAll();
    for (const auto& entry : entriesB) {
        if (!sharedQueueC.exists(entry.uid)) {
            sharedQueueC.add1(entry.uid.c_str(), entry.timestamp.c_str(), entry.number);
        }
    }

    // Add from Queue A (lower priority)
    std::vector<QueueEntry> entriesA = sharedQueue.getAll();
    for (const auto& entry : entriesA) {
        if (!sharedQueueC.exists(entry.uid)) {
            sharedQueueC.add1(entry.uid.c_str(), entry.timestamp.c_str(), entry.number);
        }
    }

    Serial.println("🔄 Rebuilt sharedQueueC (mixed queue).");
}
void printAllQueues() {
   std::vector<QueueEntry> entries = sharedQueue.getAll();
    Serial.println("📋 All Queues:");
    Serial.print("🔸 sharedQueue: "); sharedQueue.print();
    Serial.print("🔸 sharedQueueA: "); sharedQueueA.print();
    Serial.print("🔸 sharedQueueB: "); sharedQueueB.print();
    Serial.print("🔸 sharedQueueC: "); sharedQueueC.print();
    Serial.printf("Total patients in queue: %d\n", entries.size());
}

void clearAllQueues() {
    sharedQueue.clear(); sharedQueueA.clear(); sharedQueueB.clear(); sharedQueueC.clear(); sharedQueue.save();
    sharedQueueA.save();
    sharedQueueB.save();
    sharedQueueC.save();
    clearPreferences();
    Serial.println("🔄 All queues cleared.");
}
  DateTime parseDateTime(String timestamp) {
    int yr, mo, dy, hr, mn, sc;
    sscanf(timestamp.c_str(), "%d-%d-%d %d:%d:%d", &yr, &mo, &dy, &hr, &mn, &sc);
    return DateTime(yr, mo, dy, hr, mn, sc);
}
    //#include <Preferences.h>

void clearPreferences() {
    Preferences prefs;
    prefs.begin("rfidMap", false); // Open your namespace in read-write mode
    prefs.clear(); // Delete all keys in this namespace
    prefs.end();
    Serial.println("✅ Cleared all preferences in 'rfidMap' namespace");
}

 void setup() {


    Serial.begin(115200);
    prefs.begin("rfidMap", false);
    //loadRFIDMappings(prefs);  
    //clearPreferences();

    if (!LittleFS.begin(true)) { // true = format if mount fails
        Serial.println("❌ LittleFS mount failed, formatted.");
    } else {
        Serial.println("✅ LittleFS mounted.");
    }

    // Save mappings once (comment out after first run)
   //saveRFIDMappingsToFile();

     pinMode(IR_SENSOR_PIN, INPUT);
  myServo.attach(SERVO_PIN, 500, 2500);  // Correct for ESP32 + SG90
  myServo.write(0);  // Start at 0 degrees
  //clearAllQueues();
   
myIndex = 0;
for (; myIndex < numArrivalNodes; ++myIndex) {
    if (memcmp(WiFi.macAddress().c_str(), arrivalMACs[myIndex], 6) == 0) break;
}
bool isMaster = (myIndex == 0);
size_t currentMasterIndex = 0;

String myMAC = WiFi.macAddress();
Serial.print("This node MAC: "); Serial.println(myMAC);

for (int i = 0; i < numArrivalNodes; i++) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             arrivalMACs[i][0], arrivalMACs[i][1], arrivalMACs[i][2],
             arrivalMACs[i][3], arrivalMACs[i][4], arrivalMACs[i][5]);
    if (myMAC.equalsIgnoreCase(String(macStr))) {
        myIndex = i;
        break;
    }
}

isMaster = (myIndex == 0);  // Node 0 is default master
currentMasterIndex = 0;




    SPI.begin(); WiFi.mode(WIFI_STA);
WiFi.setTxPower(WIFI_POWER_19_5dBm); 
 esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);   
  Serial.print("WiFi MAC: "); Serial.println(WiFi.macAddress());
    mfrc522.PCD_Init();
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(BUZZER, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
   //  pinMode(BLUE_LED_PIN, OUTPUT);
    digitalWrite(GREEN_LED_PIN, LOW );
    digitalWrite(RED_LED_PIN, LOW);
    //digitalWrite(BLUE_LED_PIN, HIGH);
    if (!rtc.begin()) {
        Serial.println("❌ Couldn't find RTC module! Check wiring.");
        while (1);
    }

    if (rtc.lostPower()) {
        Serial.println("⚠️ RTC lost power, setting to compile time.");
        rtc.adjust(DateTime(__DATE__, __TIME__));
    }

    if (esp_now_init() != ESP_OK) {
        Serial.println("❌ ESP-NOW Init Failed");
        return;
    }

    for (int i = 0; i < numArrivalNodes; i++) {
        esp_now_peer_info_t p = {};
        memcpy(p.peer_addr, arrivalMACs[i], 6);
        p.channel = 1;
        esp_now_add_peer(&p);
    }

    for (int i = 0; i < 4; i++) {
        esp_now_peer_info_t p = {};
        memcpy(p.peer_addr, doctorMACs[i], 6);
        p.channel = 1;
        esp_now_add_peer(&p);
    }

    esp_now_register_send_cb(OnDataSent);
   esp_now_register_recv_cb(onDataRecv);

 if (isMaster) {
    delay(100);  // small delay to ensure peers are ready (optional)
    sendTimeToAllArrivalNodes();  // master sends current RTC time to all others
    }
    sharedQueue.load();
    sharedQueueA.load();
    sharedQueueB.load();
    sharedQueueC.load();
if(arbiter){
createMixedQueue();broadcastAllQueues(); //Only arbiter should contain the following  
}
     //printAllQueues();
pinMode(IR_SENSOR_PIN, INPUT);
myServo.attach(SERVO_PIN, 500, 2500);  // Correct for ESP32 + SG90
myServo.write(0);// digitalWrite(BLUE_LED_PIN , HIGH);
}



//bool waitingForCard = false;

void loop() {
  if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        if (cmd == "validate") {
            validateQueues();
        }
    }
 static unsigned long lastValidationTime = 0;
    const unsigned long VALIDATION_INTERVAL = 30000; // 30 seconds
    if (millis() - lastValidationTime >= VALIDATION_INTERVAL) {
        validateQueues();
        lastValidationTime = millis();
    }
 static unsigned long lastSyncTime = 0;
    unsigned long nowMillis = millis();
    if (isMaster && nowMillis - lastSyncTime >= 3600000UL) { // Every hour
        sendTimeToAllArrivalNodes();
        lastSyncTime = nowMillis;
    }
     static unsigned long lastHousekeeping = 0;
    unsigned long now = millis();
    if (now - lastHousekeeping > 100) {
        manageRetries();
        lastHousekeeping = now;
    }
 
    // Check for object using IR sensor
    int sensorValue = digitalRead(IR_SENSOR_PIN); delay(50);
    if (sensorValue == LOW) {
      Serial.println("🚫 Object detected by IR sensor.");
      Serial.println("🔄 Dispensing card...");

      myServo.write(65);  // Open or dispense
      delay(1000);        // Let the card fall
      myServo.write(0);   // Close again
      delay(500);         // Optional debounce delay

      waitingForCard = true;  // Now expect card scan
    }
  

    // Wait for RFID card scan
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
      String uid = getUIDString(mfrc522.uid.uidByte, mfrc522.uid.size);
      k=1; 
      if(arbiter){ processCard(uid);
      
      mfrc522.PICC_HaltA(); 
      mfrc522.PCD_StopCrypto1(); 
      delay(1200);

      createMixedQueue(); broadcastAllQueues();//Only arbiter should contain the following

   
      }
      else{ processCard(uid);
      mfrc522.PICC_HaltA(); 
      mfrc522.PCD_StopCrypto1(); 
      delay(1200);
      // createMixedQueue();
      }                   //Only arbiter should contain the following
     
      k=0;                    //Only arbiter should contain the following
      //printAllQueues();
      waitingForCard = false;  // Return to sensor check
    }
  

    static unsigned long lastSyncTime1 = 0;
unsigned long nowMillis1 = millis();
if (isMaster && nowMillis1 - lastSyncTime1 >= 7200000UL) {  // 2 hours = 7200000 ms
    sendTimeToAllArrivalNodes();
    lastSyncTime1 = nowMillis1;
}
 if (isTimeToClearQueues() && !alreadyClearedToday()) {
        if (isMaster) {
            Serial.println("🕒 4AM detected - Clearing all queues");
            clearAllQueues();
            
            // Broadcast clear command to all arrival nodes
            QueueItem clearCmd;
            strcpy(clearCmd.uid, "CLEAR_QUEUE");
            clearCmd.addToQueue = false;
            clearCmd.removeFromQueue = true;
            
            for (int i = 0; i < numArrivalNodes; ++i) {
                if (i == myIndex) continue;
                esp_now_send(arrivalMACs[i], (uint8_t*)&clearCmd, sizeof(clearCmd));
            }
            
            // Notify doctors
            QueueItem noPat{};
            strncpy(noPat.uid, "NO_PATIENT", sizeof(noPat.uid));
            noPat.number = 0;
            noPat.addToQueue = false;
            noPat.removeFromQueue = false;
            
            for (int i = 0; i < numDoctorNodes; ++i) {
                noPat.node = i + 1;
                esp_now_send(doctorMACs[i], (uint8_t*)&noPat, sizeof(noPat));
            }
        }
    }


}

String getUIDString(byte *buf, byte size) {
    String uid=""; for (byte i=0;i<size;i++)
    {if(buf[i]<0x10)uid+="0"; uid+=String(buf[i],HEX);} uid.toUpperCase(); return uid;
}

void blinkLED(int pin) {
    //digitalWrite(BLUE_LED_PIN , LOW);
    digitalWrite(pin, HIGH); //digitalWrite(BUZZER,HIGH ); 
    delay(500); 
    //digitalWrite(BLUE_LED_PIN , HIGH);
    digitalWrite(pin, LOW); //digitalWrite(BUZZER, LOW );
}


