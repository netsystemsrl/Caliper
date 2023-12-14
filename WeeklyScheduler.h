#include <TimeLib.h> //dovrebbe fare anche syncro dell rtc / gps /ntp (ce da guardare come fa)
#include <ArduinoJson.h>
#define MAX_TASKS 20
#include <Preferences.h>
Preferences preferences;


String padTime(byte numberHH, byte numberMM) {
  String Result = ""; 
  if (numberHH<10) 
    Result = Result + "0" + numberHH;
  else
    Result = Result + numberHH;
  Result = Result + ':';
  if (numberMM<10) 
    Result = Result + "0" + numberMM;
  else
    Result = Result + numberMM;
  return Result;
}

typedef void( * TaskCallback)(byte);
struct Task {
  byte dayOfWeek;
  byte startHour;
  byte startMinute;
  byte endHour;
  byte endMinute;
  unsigned long repeatEvery; // Intervallo di ripetizione in seconds
  unsigned long duration; // Intervallo in seconds
  String callbackName; // Change type to String
};

class WeeklyScheduler {
  public: Task tasks[MAX_TASKS];
  public: bool eprom = true;
  unsigned long lastExecutionTime[MAX_TASKS] = { 0 };

  /* INITIALIZE */
  public: void begin() {
    for (byte i = 0; i < MAX_TASKS; i++) {
      tasks[i].dayOfWeek = 0;
      tasks[i].startHour = 0;
      tasks[i].startMinute = 0;
      tasks[i].endHour = 0;
      tasks[i].endMinute = 0;
      tasks[i].repeatEvery = 0;
      tasks[i].duration = 0;
      tasks[i].callbackName = ""; // Assign an empty string instead of NULL
    }
    if (eprom) loadFromEprom();
  }

  /* MANAGE TASKS */
  public: void addTask(byte dayOfWeek, byte startHour, byte startMinute, byte endHour, byte endMinute, unsigned long repeatEvery, unsigned long duration, const String & callbackName) {

    for (byte i = 0; i < MAX_TASKS; i++) {
      if (tasks[i].callbackName == "") {
        tasks[i].dayOfWeek = dayOfWeek;
        tasks[i].startHour = startHour;
        tasks[i].startMinute = startMinute;
        tasks[i].endHour = endHour;
        tasks[i].endMinute = endMinute;
        tasks[i].repeatEvery = repeatEvery;
        tasks[i].duration = duration;
        tasks[i].callbackName = callbackName;
        
        Serial.println("Scheduler SAVE EPROM");
        preferences.begin("taskslist" + i, false);    //true = write protect

        preferences.putUChar("dayOfWeek", dayOfWeek);
        preferences.putUChar("startHour", startHour);
        preferences.putUChar("startMinute", startMinute);
        preferences.putUChar("endHour", endHour);
        preferences.putUChar("endMinute", endMinute);
        preferences.putULong("repeatEvery", repeatEvery);
        preferences.putULong("duration", duration);
        preferences.putString("callbackName", callbackName);
        preferences.end();
        
        break;
      }
    }
  }

  public: void addTask(byte dayOfWeek, const String& startTime, const String& endTime, unsigned long repeatInterval, unsigned long duration, const String& callbackName) {
    // Parse the start time
    byte startHour = startTime.substring(0, 2).toInt();
    byte startMinute = startTime.substring(3, 5).toInt();

    // Parse the end time
    byte endHour = endTime.substring(0, 2).toInt();
    byte endMinute = endTime.substring(3, 5).toInt();

    // Call the existing addTask method with the parsed values
    addTask(dayOfWeek, startHour, startMinute, endHour, endMinute, repeatInterval, duration, callbackName);
  }

  public: void deleteAllTasks() {
    
    for (byte i = 0; i < MAX_TASKS; i++) {
      tasks[i].dayOfWeek = 0;
      tasks[i].startHour = 0;
      tasks[i].startMinute = 0;
      tasks[i].endHour = 0;
      tasks[i].endMinute = 0;
      tasks[i].repeatEvery = 0;
      tasks[i].duration = 0;
      tasks[i].callbackName = "";
      if (eprom) {
        preferences.begin("taskslist" + i, false);
        preferences.clear();
        preferences.end();
      }
    }
  }

  /* MAIN LOOP */
  public: void run() {
    byte currentDayOfWeek = rtc.getDayofWeek();
    byte currentHour = rtc.getHour(true);
    byte currentMinute = rtc.getMinute();
    //byte currentSecond = rtc.getSecond();

    for (byte i = 0; i < MAX_TASKS; i++) {
      Task task = tasks[i];
      if (task.callbackName != "" && task.dayOfWeek == currentDayOfWeek) {

        tmElements_t startTimeElements;
        startTimeElements.Hour = task.startHour;
        startTimeElements.Minute = task.startMinute;
        startTimeElements.Second = 0; //task.startSecond;
        startTimeElements.Day = rtc.getDay();
        startTimeElements.Month = rtc.getMonth();
        startTimeElements.Year = rtc.getYear();
        time_t startTime = makeTime(startTimeElements);

        tmElements_t endTimeElements;
        endTimeElements.Hour = task.endHour;
        endTimeElements.Minute = task.endMinute;
        endTimeElements.Second = 0; //task.endSecond;
        endTimeElements.Day = rtc.getDay();
        endTimeElements.Month = rtc.getMonth();
        endTimeElements.Year = rtc.getYear();
        time_t endTime = makeTime(endTimeElements);

        tmElements_t taskTimeElements;
        taskTimeElements.Hour = currentHour;
        taskTimeElements.Minute = currentMinute;
        taskTimeElements.Second = 0; //currentTime.second();
        taskTimeElements.Day = rtc.getDay();
        taskTimeElements.Month = rtc.getMonth();
        taskTimeElements.Year = rtc.getYear();
        time_t taskTime = makeTime(taskTimeElements);

        if ( (taskTime >= startTime) 
            && (taskTime <= endTime)
         ) {
          if (!task.repeatEvery) {
            WeeklySchedulerFunction(task.callbackName, task.duration, task.repeatEvery);
          } 
          else {
            if (millis() - lastExecutionTime[i] >= (task.repeatEvery)*1000) {
              lastExecutionTime[i] = millis();
              WeeklySchedulerFunction(task.callbackName, task.duration, task.repeatEvery);
            }
          }
        }
      }
    }
  }

  /* JSON SERIALIZE UNSERIALIZE */
  public: String toJSON() {
    DynamicJsonDocument doc(2048);
    JsonArray taskArray = doc.createNestedArray("tasks");

    for (byte i = 0; i < MAX_TASKS; i++) {
      Task task = tasks[i];
      if (task.callbackName != "") {
        JsonObject taskObject = taskArray.createNestedObject();
        taskObject["D"] = task.dayOfWeek;
        taskObject["S"] = padTime(task.startHour, task.startMinute); 
        taskObject["E"] = padTime(task.endHour, task.endMinute);
        taskObject["R"] = task.repeatEvery;
        taskObject["T"] = task.duration;
        taskObject["N"] = task.callbackName;
      }
    }

    String jsonString;

    //DAFARE SCAMBIARE X ULTIMO PER OTTIMIZZZARE
    //serializeJson(doc, jsonString);
    serializeJsonPretty(doc, jsonString);
    return jsonString;
  }

  public: void fromJSON(const String & jsonString) {
    DynamicJsonDocument doc(4096);
    DeserializationError error = deserializeJson(doc, jsonString);

    if (error) {
      Serial.print("Scheduler Failed to parse JSON: ");
      Serial.println(error.c_str());
      return;
    }

    JsonArray taskArray = doc["tasks"];
    deleteAllTasks();

    for (const auto & taskObject: taskArray) {
      byte dayOfWeek = taskObject["D"];
      String start = taskObject["S"];
      String end = taskObject["E"];
      unsigned long repeatEvery = taskObject["R"];
      unsigned long duration = taskObject["T"];
      String callbackName = taskObject["N"];

      // Create a new task
      addTask(dayOfWeek, start, end, repeatEvery, duration, callbackName);
    }
  }

  private: void loadFromEprom(){

    for (byte i = 0; i < MAX_TASKS; i++) {

      preferences.begin("taskslist" + i, false);    //true = write protect
      tasks[i].dayOfWeek = preferences.getUChar("dayOfWeek", 0);
      tasks[i].startHour = preferences.getUChar("startHour", 0);
      tasks[i].startMinute = preferences.getUChar("startMinute", 0);
      tasks[i].endHour = preferences.getUChar("endHour", 0);
      tasks[i].endMinute = preferences.getUChar("endMinute", 0);
      tasks[i].repeatEvery = preferences.getULong("repeatEvery", 0);
      tasks[i].duration = preferences.getULong("duration", 0);
      tasks[i].callbackName = preferences.getString("callbackName", "");
      preferences.end();

    }
  }

};