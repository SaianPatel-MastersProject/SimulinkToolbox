#pragma once

#include <set>
#include <unordered_map>

#include "StringUtils.hpp"
#include "rFProTypes.h"

// Actions:
//   add a mesh to list of those available
//      - allows update of meshes to send
//   add a new config group (list of mesh names)
//   update list of mesh IDs to send
//   generate a list of mesh IDs for particular config group
//      - to send to external model
//      - to update list of IDs of meshes that need to be sent
//   generate messages of BBs to send
class BoundingBoxControl
{
public:
  // This maximum value would mean 1996 bytes in a message, which is within what the
  // Simulink library can handle, but is still large for a UDP packet.
  // (The SL library also limits the # bounding boxes to 19 - as of 28/09/23)
  static const int MAX_BOXES = 19;

  BoundingBoxControl()
  {
    mesh_IDs_.clear();
    bb_groups_.clear();
    defined_groups_.clear();
    memset(&message_buffer_, 0, sizeof(UDPBoundBoxListV02) + MAX_BOXES*sizeof(UDPBoundBoxV01));
    last_message_size_ = 0;
    // Put 255 values here to indicate that we are using UDPBoundBoxListV02, rather than V01,
    // which is not a perfect method, as the data was not set in V01.
    // When we drop support for V01 we can use this space for something else.
    message_buffer_.header.mUnused[0] = 255;
    message_buffer_.header.mUnused[1] = 255;
    message_buffer_.header.mUnused[2] = 255;
    message_buffer_.header.mUnused[3] = 255;
  }

  // On receiving a config message listing wanted BBs with wildcards
  // ---------------------------------------------------------------
  void add_new_group(uint16_t group_id, const std::vector<std::string>& new_config)
  {
    bb_groups_[group_id].group_names_.clear();
    bb_groups_[group_id].bb_vector_.reserve(MAX_BOXES*25);
    std::fill(bb_groups_[group_id].bb_vector_.begin(), bb_groups_[group_id].bb_vector_.end(), 0);
    
    for (auto item = new_config.begin(); item != new_config.end(); ++item)
    {
      bb_groups_[group_id].group_names_.emplace_back(*item);
    }

    // Convert names to IDs and update group list
    bb_groups_[group_id].group_mesh_ids_ = find_group_IDs(group_id);
    defined_groups_.emplace_back(group_id);
  }
  
  void add_new_group(uint16_t group_id, const std::vector<UDPBoundBoxConfigItemV01>& new_config)
  {
    std::vector<std::string> names;
    names.reserve(new_config.size());
    for (auto config_item = new_config.begin(); config_item != new_config.end(); ++config_item)
    {
      names.emplace_back(config_item->mInstanceName);
    }
    add_new_group(group_id, names);
  }

  // New mesh available as a result of call to WantsBoundingBox
  // ----------------------------------------------------------
  void add_new_mesh(const QueryInstance& instance, bool include_children)
  {
    // Add mesh to the map of names to IDs 
    if (instance.mNumMeshes > 0 && (include_children || instance.mParent == nullptr))
    {
      mesh_IDs_.emplace(std::pair<const std::string, uint32_t>(std::string(instance.mName), instance.mMesh[0].mID));
    }

    // Check each of the mesh groups to see if there are any changes
    if (!bb_groups_.empty())
    {
      for (auto group = bb_groups_.begin(); group != bb_groups_.end(); ++group)
      {
        std::set<uint32_t> new_ids = find_group_IDs(group->first);
        if (new_ids != group->second.group_mesh_ids_)
        {
          group->second.group_mesh_ids_.swap(new_ids);
        }
      }
    }
  }

  // Put bounding boxes for specified group into object's buffer and update message size
  // -----------------------------------------------------------------------------------
  const char* generate_udp_message(const uint16_t group_number, const long num, const BoundingBoxV01* bb_array)
  {
    // Prepare header
    message_buffer_.header.messageType = RFPRO_BOUNDING_BOX;
    message_buffer_.header.mLength = static_cast<uint16_t>(bb_groups_[group_number].group_mesh_ids_.size());
    message_buffer_.header.mVersion = 1;
    message_buffer_.header.mMeshGroup = group_number;

    // Loop over group indices, adding bb
    uint16_t counter = 0;
    for (long bb_idx = 0; bb_idx < num && counter < MAX_BOXES; ++bb_idx)
    {
      if (bb_groups_[group_number].group_mesh_ids_.find(bb_array[bb_idx].mMeshID) != bb_groups_[group_number].group_mesh_ids_.end())
      {
        if (bb_array[bb_idx].m3DExtentsValid)
        {
          //mMessageBuffer.mBBox[counter].mMeshID
          message_buffer_.b_box[counter].mMeshID = bb_array[bb_idx].mMeshID;
          message_buffer_.b_box[counter].m3DExtentsValid = bb_array[bb_idx].m3DExtentsValid;
          for (int j = 0; j < 3; ++j)
          {
            // We only send 4 pts that allow the receiver to generate the other 4 corners
            message_buffer_.b_box[counter].mWorld3D[0][j] = bb_array[bb_idx].mWorld3D[0][j];
            message_buffer_.b_box[counter].mWorld3D[1][j] = bb_array[bb_idx].mWorld3D[1][j];
            message_buffer_.b_box[counter].mWorld3D[2][j] = bb_array[bb_idx].mWorld3D[3][j];
            message_buffer_.b_box[counter].mWorld3D[3][j] = bb_array[bb_idx].mWorld3D[4][j];
          }
          ++counter;
        }
      }
    }

    message_buffer_.header.mLength = counter;

    if (counter == 0)
    {
      last_message_size_ = 0;
    }
    else
    {
      last_message_size_ = sizeof(UDPBoundBoxListV02) + counter * sizeof(UDPBoundBoxV01);
    }

    return reinterpret_cast<const char*>(&message_buffer_);
  }

  void generate_group_vector(const uint16_t group_number, const long num, const BoundingBoxV01* bb_array)
  {
    // Write the required bounding box data to a vector in a map
    // Get a reference to the prepared vector - whose capacity is set in the constructor
    std::vector<double>& buffer = bb_groups_[group_number].bb_vector_;
    std::fill(buffer.begin(), buffer.end(), 0);
    int counter = 0;
    for (long bb_idx = 0; bb_idx < num && counter < MAX_BOXES; ++bb_idx)
    {
      if (bb_groups_[group_number].group_mesh_ids_.find(bb_array[bb_idx].mMeshID) != bb_groups_[group_number].group_mesh_ids_.end())
      {
        if (bb_array[bb_idx].m3DExtentsValid)
        {
          buffer[counter] = bb_array[bb_idx].mMeshID;
          for (int corner = 0; corner < 8; ++corner)
          {
            int idx = MAX_BOXES + 24*counter + 3*corner;
            for (int j = 0; j < 3; ++j)
            {
              buffer[idx++] = bb_array[bb_idx].mWorld3D[corner][j];
            }
          }
          ++counter;
        }
      }
    }  
  }
  
  // Access functions
  size_t num_groups() const { return bb_groups_.size(); }
  size_t get_last_message_size() const { return last_message_size_; }
  const std::vector<uint16_t>& get_group_ids() { return defined_groups_; }
  double* get_group_vector(const uint16_t group_id) {return bb_groups_[group_id].bb_vector_.data();};
 
private: // functions

  // For a specified group find all the matching mesh IDs.
  std::set<uint32_t> find_group_IDs(uint16_t group_number)
  {
    std::set<uint32_t> result;
    result.clear();
    for (auto config_name = bb_groups_[group_number].group_names_.begin(); config_name != bb_groups_[group_number].group_names_.end(); ++config_name)
    {
      // If it were not for the wildcards we could just use the map key to get a
      // single ID directly.
      for (auto current_mesh = mesh_IDs_.begin(); current_mesh != mesh_IDs_.end(); ++current_mesh)
      {
        if (StringUtils::WildcardMatch(current_mesh->first.c_str(), config_name->c_str()))
        {
          result.insert(current_mesh->second);
        }
      }
    }
    return result;
  }

private: // variables

  struct box_group
  {
    std::vector<std::string> group_names_;
    std::set<uint32_t> group_mesh_ids_;
    std::vector<double> bb_vector_;
  };

  std::unordered_map<std::string, uint32_t> mesh_IDs_;
  std::unordered_map<uint16_t, box_group> bb_groups_;
  std::vector<uint16_t> defined_groups_;

  size_t last_message_size_;

#pragma pack( push, 4 )

  struct bb_message
  {
    UDPBoundBoxListV02 header;
    UDPBoundBoxV01 b_box[MAX_BOXES];
  }  message_buffer_;

#pragma pack( pop )

};
