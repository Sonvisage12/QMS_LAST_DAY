#ifndef SHAREDQUEUE_H
#define SHAREDQUEUE_H

#include <Arduino.h>
#include <vector>
#include <Preferences.h>
#include <RTClib.h>

struct QueueEntry {
  String uid;
  String timestamp;
  int number;
};

struct QueueItem {  // For ESP-NOW communication
  char uid[20];
  char type[10]; 
  char timestamp[25];
  int number;
  int node;
  bool removeFromQueue;
  bool addToQueue;
  char queueID[10];
  char sourceMAC[18];
};

class SharedQueue {
public:
  SharedQueue(const String& ns);
  void load();
  void save();
  void print();
  void clear();
  void add(const String& uid, const String& timestamp, int number);
  void add1(const String& uid, const String& timestamp, int number);
  void addIfNew(const String& uid, const String& timestamp, int number);
  void removeByUID(const String& uid);
  bool exists(const String& uid);
  int getOrAssignPermanentNumber(const String& uid, const DateTime& now);
//void insertAt(int position, const String& uid, const String& timestamp, int number);
  bool empty() const { return queue.empty(); }
  QueueEntry& front() { return queue.front(); }
  void pop() { queue.erase(queue.begin()); }
  void push(const QueueEntry& item) { queue.push_back(item); }
void insertAt(int position, const String& uid, const String& timestamp, int number) {
    if (position < 0 || position > queue.size()) position = queue.size();  // Clamp to end

    QueueEntry newEntry = {uid, timestamp, number};
    queue.insert(queue.begin() + position, newEntry);
    save();  // Ensure the new queue is saved
}


  QueueEntry getEntry(String uid);
  std::vector<QueueEntry>& getQueue();
  void sortQueue();  // 🟢 Make sortQueue public


    std::vector<QueueEntry> getAll() {
        std::vector<QueueEntry> copyQueue = queue;  // Copy internal vector
        return copyQueue;
    }

private:
  std::vector<QueueEntry> queue;      // 🔥 Use vector here
 
  String name;
  Preferences prefs;
  String namespaceStr;
  int counter;
};

#endif
