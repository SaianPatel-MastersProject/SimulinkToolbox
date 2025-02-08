#pragma once

#include <list>

class TelemetryOpponentList
{
public:
  TelemetryOpponentList()
  {
    vehicleTelem_.clear();
  }

  // Given telemetry from UpdateTelemetry add distance to ego vehicle and slot ID.
  void addTelemetry(TelemInfoV01 data)
  {
    if (data.mID == egoID_)
      return;

    // Find and remove existing telemetry
    for (std::list<carItem>::iterator item = vehicleTelem_.begin(); item != vehicleTelem_.end(); )
    {
        if ((*item).slotID == data.mID)
            item = vehicleTelem_.erase(item);
        else
            ++item;
    }
    
    carItem newTelem;
    newTelem.distance = (data.mPos.x - egoPos_.x)*(data.mPos.x - egoPos_.x)
                      + (data.mPos.y - egoPos_.y)*(data.mPos.y - egoPos_.y) 
                      + (data.mPos.z - egoPos_.z)*(data.mPos.z - egoPos_.z) ;
    newTelem.slotID = data.mID;
    newTelem.telemetry = data;

    vehicleTelem_.push_back(newTelem);
  }

  TelemInfoV01* find_telemetry(bool nearest, int required_id)
  {
    int count = 0;
    std::list<carItem>::iterator item;
    for (item = vehicleTelem_.begin(); item != vehicleTelem_.end(); ++item)
    {
      if (nearest && count == required_id)
        break;
      if (!nearest && (*item).slotID == required_id)
        break;
      ++count;
    }

    return item != vehicleTelem_.end() ? &((*item).telemetry) : nullptr;
  }
      
  void prepare()
  {
    if (telemetry_active_)
      vehicleTelem_.sort();
  }
  
  void reset()
  {
    if (telemetry_active_)
      vehicleTelem_.clear();
  }
  
  // Setters:
  // ego ID to block that data from list
  // ego position for calculating relative distances
  void toggleTelemetry(bool new_value) { vehicleTelem_.clear(); telemetry_active_ = new_value; };
  void setEgoID(int id) { egoID_ = id; };
  void setEgoPosition(TelemVect3 pos) { egoPos_.Set(pos.x, pos.y, pos.z); };

  // Getters
  bool active() { return telemetry_active_; }
  bool getEgoID() { return egoID_; };
  int size() {return vehicleTelem_.size(); }

private:
  struct carItem
  {
    int slotID = 0;
    double distance = 0.0;
    TelemInfoV01 telemetry;
    bool operator< (const carItem& rhs) const
    {
      return this->distance < rhs.distance;
    }
  };

  // Private variables
  std::list<carItem> vehicleTelem_; // Current vehicle telemetries - to output
  bool telemetry_active_ = false;

  // Ego vehicle telemetry - to sort opponents by distance to ego
  int egoID_ = -1;
  double maxDist_ = 0.0;
  TelemVect3 egoPos_ = {0.0, 0.0, 0.0 };
};