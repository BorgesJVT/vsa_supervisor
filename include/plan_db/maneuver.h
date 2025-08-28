#ifndef __SUPERVISOR_PLANDB_MANUEVER__
#define __SUPERVISOR_PLANDB_MANUEVER__

#include <string>
#include <json/json.h>
#include "neptus_msgs/msg/maneuver.hpp"
#include "neptus_msgs/msg/plan_maneuver.hpp"
#include "../guidance/types.h"

enum maneuver_speed_units_e
{
    maneuver_speed_unit_meters_ps = 0,
    maneuver_speed_unit_rpm,
    maneuver_speed_unit_percentage,
    maneuver_speed_unit_size
};

enum maneuver_z_units_e
{
    maneuver_z_unit_none = 0,
    maneuver_z_unit_depth,
    maneuver_z_unit_altitude,
    maneuver_z_unit_height,
    maneuver_z_unit_size
};

class Maneuver
{
    public:
        Maneuver();
        Maneuver(neptus_msgs::msg::PlanManeuver plan_maneuver);
        Maneuver(Json::Value maneuver);
        ~Maneuver();

    public:
        Json::Value serialize_json();
        void deserialize_json(Json::Value maneuver);

    public:
        std::string id;
        std::string name;
        uint32_t maneuver_imc_id;

        double lat;
        double lon;

        double z;
        maneuver_z_units_e z_unit;
        double speed;
        maneuver_speed_units_e speed_unit;

        double roll;
        double pitch;
        double yaw;

        uint32_t timeout;
        std::string custom_string;

        uint32_t syringe0;
        uint32_t syringe1;
        uint32_t syringe2;

        std::vector<vsa_guidance::polygon_vertex_t> polygon;
    
    private:
        Json::Value maneuver_;


};

#endif