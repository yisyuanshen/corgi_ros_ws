#ifndef CONTACT_PROCESSOR_H
#define CONTACT_PROCESSOR_H

#include <map>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include "geometry_msgs/Point.h"
#include "corgi_msgs/SimContactPoint.h"
#include "corgi_msgs/StructuredContactDataStamped.h"
#include "corgi_msgs/LegContact.h"
#include "corgi_msgs/RimContact.h"
#include "corgi_msgs/ContactPoint.h"

class ContactProcessor {
private:
    std::map<std::string, std::string> name_mapping;
    std::vector<std::string> leg_names;
    std::vector<std::string> rim_names;
    
    void initializeMappings();

public:
    ContactProcessor();
    ~ContactProcessor() = default;
    
    std::string extractLegName(const std::string& contact_name);
    std::string mapRimName(const std::string& original_name);
    
    corgi_msgs::StructuredContactDataStamped processContacts(const std::vector<corgi_msgs::SimContactPoint>& raw_contacts);
    
    // Query functions
    bool hasLegContact(const corgi_msgs::StructuredContactDataStamped& data, const std::string& leg_name);
    bool hasRimContact(const corgi_msgs::StructuredContactDataStamped& data, const std::string& leg_name, const std::string& rim_name);
    std::vector<geometry_msgs::Point> getRimContactPoints(const corgi_msgs::StructuredContactDataStamped& data, 
                                                          const std::string& leg_name, 
                                                          const std::string& rim_name);
};

#endif // CONTACT_PROCESSOR_H