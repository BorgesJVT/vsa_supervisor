#include "../../include/plan_db/maneuver.h"

Maneuver::Maneuver()
{

}

Maneuver::Maneuver(neptus_msgs::msg::PlanManeuver maneuver)
{
    id = maneuver.maneuver_id;

    name = maneuver.maneuver.maneuver_name;
    maneuver_imc_id = maneuver.maneuver.maneuver_imc_id;
    
    lat = maneuver.maneuver.lat;
    lon = maneuver.maneuver.lon;
    z = maneuver.maneuver.z;
    z_unit = (maneuver_z_units_e)maneuver.maneuver.z_units;
    speed = maneuver.maneuver.speed;
    speed_unit = (maneuver_speed_units_e)maneuver.maneuver.speed_units;
    
    roll = maneuver.maneuver.roll;
    pitch = maneuver.maneuver.pitch;
    yaw = maneuver.maneuver.yaw;

    timeout = maneuver.maneuver.timeout;
    custom_string = maneuver.maneuver.custom_string;

    syringe0 = maneuver.maneuver.syringe0;
    syringe1 = maneuver.maneuver.syringe1;
    syringe2 = maneuver.maneuver.syringe2;

    for(int i = 0; i < maneuver.maneuver.polygon.size(); i++)
    {
        vsa_guidance::polygon_vertex_t vertex;

        vertex.lat = maneuver.maneuver.polygon[i].lat;
        vertex.lon = maneuver.maneuver.polygon[i].lon;

        polygon.push_back(vertex);
    }
}

Maneuver::Maneuver(Json::Value maneuver)
{
    deserialize_json(maneuver);
}

Maneuver::~Maneuver()
{

}

Json::Value Maneuver::serialize_json()
{
    maneuver_["maneuver_id"] = id;
    maneuver_["maneuver_imc_id"] = maneuver_imc_id;
    maneuver_["lat"] = lat;
    maneuver_["lon"] = lon;
    maneuver_["z"] = z;
    maneuver_["z_units"] = (uint8_t)z_unit;
    maneuver_["speed"] = speed;
    maneuver_["speed_units"] = (uint8_t)speed_unit;
    maneuver_["roll"] = roll;
    maneuver_["pitch"] = pitch;
    maneuver_["yaw"] = yaw;
    maneuver_["timeout"] = timeout;
    maneuver_["custom_string"] = custom_string;
    maneuver_["syringe0"] = syringe0;
    maneuver_["syringe1"] = syringe1;
    maneuver_["syringe2"] = syringe2;

    Json::Value _polygon(Json::arrayValue);

    for(int i = 0; i < polygon.size(); i++)
    {
        Json::Value vertex;

        vertex["lat"] = polygon[i].lat;
        vertex["lon"] = polygon[i].lon;

        _polygon.append(vertex);
    }

    maneuver_["polygon"] = _polygon;

    return maneuver_;
}
        
void Maneuver::deserialize_json(Json::Value maneuver)
{
    polygon.clear();

    id = maneuver["maneuver_id"].asString();
    maneuver_imc_id = (uint32_t)maneuver["maneuver_imc_id"].asUInt64();
    lat = maneuver["lat"].asDouble();
    lon = maneuver["lon"].asDouble();
    z = maneuver["z"].asDouble();
    z_unit = (maneuver_z_units_e)maneuver["z_units"].asUInt64();
    speed = maneuver["speed"].asDouble();
    speed_unit = (maneuver_speed_units_e)maneuver["speed_units"].asUInt64();
    roll = maneuver["roll"].asDouble();
    pitch = maneuver["pitch"].asDouble();
    yaw = maneuver["yaw"].asDouble();
    timeout = (uint32_t)maneuver["timeout"].asUInt64();
    custom_string = maneuver["custom_string"].asString();
    syringe0 = (uint32_t)maneuver["syringe0"].asUInt64();
    syringe1 = (uint32_t)maneuver["syringe1"].asUInt64();
    syringe2 = (uint32_t)maneuver["syringe2"].asUInt64();

    for (auto itr : maneuver["polygon"])
    {
        vsa_guidance::polygon_vertex_t vertex;
        vertex.lat = itr["lat"].asDouble();
        vertex.lon = itr["lon"].asDouble();
    }
}