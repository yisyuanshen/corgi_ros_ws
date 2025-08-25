#include "contact_processor.h"

ContactProcessor::ContactProcessor() {
    initializeMappings();
}

void ContactProcessor::initializeMappings() {
    // Initialize name mapping
    name_mapping = {
        // Left Front (lf)
        {"lf_lupframe", "upper_left"},
        {"lf_rupframe", "upper_right"}, 
        {"lf_ldownframe", "lower_left"},
        {"lf_rdownframe", "lower_right"},
        {"force_lf", "G_point"},
        
        // Right Front (rf)
        {"rf_lupframe", "upper_left"},
        {"rf_rupframe", "upper_right"},
        {"rf_ldownframe", "lower_left"},
        {"rf_rdownframe", "lower_right"},
        {"force_rf", "G_point"},
        
        // Right Hind (rh)
        {"rh_lupframe", "upper_left"},
        {"rh_rupframe", "upper_right"},
        {"rh_ldownframe", "lower_left"},
        {"rh_rdownframe", "lower_right"},
        {"force_rh", "G_point"},
        
        // Left Hind (lh)
        {"lh_lupframe", "upper_left"},
        {"lh_rupframe", "upper_right"},
        {"lh_ldownframe", "lower_left"},
        {"lh_rdownframe", "lower_right"},
        {"force_lh", "G_point"}
    };
    
    leg_names = {"lf", "rf", "rh", "lh"};
    rim_names = {"upper_left", "upper_right", "lower_left", "lower_right", "G_point"};
}

std::string ContactProcessor::extractLegName(const std::string& contact_name) {
    if (contact_name.empty()) {
        return "";
    }
    
    // Handle force sensor names (force_lf -> lf)
    if (contact_name.find("force_") == 0 && contact_name.length() > 6) {
        return contact_name.substr(6);
    }
    
    // Handle frame names (lf_lupframe -> lf)
    size_t underscore_pos = contact_name.find('_');
    if (underscore_pos != std::string::npos) {
        std::string leg_part = contact_name.substr(0, underscore_pos);
        // Only return if it's a valid leg name
        if (std::find(leg_names.begin(), leg_names.end(), leg_part) != leg_names.end()) {
            return leg_part;
        }
    }
    
    return ""; // Unknown leg name
}

std::string ContactProcessor::mapRimName(const std::string& original_name) {
    if (original_name.empty()) {
        return "";
    }
    
    auto it = name_mapping.find(original_name);
    return (it != name_mapping.end()) ? it->second : "";
}

corgi_msgs::StructuredContactDataStamped ContactProcessor::processContacts(const std::vector<corgi_msgs::SimContactPoint>& raw_contacts) {
    corgi_msgs::StructuredContactDataStamped structured_data;

    // Initialize all legs and rims to no contact
    for (int i = 0; i < 4; i++) {
        structured_data.legs[i].leg_name = leg_names[i];
        structured_data.legs[i].has_contact = false;

        structured_data.legs[i].rims.resize(rim_names.size());
        
        for (int j = 0; j < 5; j++) {
            structured_data.legs[i].rims[j].rim_name = rim_names[j];
            structured_data.legs[i].rims[j].has_contact = false;
            structured_data.legs[i].rims[j].contact_points.clear();
        }
    }
    
    int total_points = 0;

    if (raw_contacts.empty()) {
        structured_data.total_contact_points = 0;
        return structured_data;
    }
    
    // Process each contact point
    for (const auto& contact : raw_contacts) {
        std::string leg_name = extractLegName(contact.name);
        std::string rim_name = mapRimName(contact.name);

        // Skip unknown contacts silently
        if (leg_name.empty() || rim_name.empty()) {
            continue;
        }
        
        // Find leg index
        auto leg_it = std::find(leg_names.begin(), leg_names.end(), leg_name);
        if (leg_it == leg_names.end()) continue;
        int leg_idx = std::distance(leg_names.begin(), leg_it);
        
        // Find rim index
        auto rim_it = std::find(rim_names.begin(), rim_names.end(), rim_name);
        if (rim_it == rim_names.end()) continue;
        int rim_idx = std::distance(rim_names.begin(), rim_it);
        
        // Skip contacts with invalid positions
        if (std::isnan(contact.point.x) || std::isnan(contact.point.y) || std::isnan(contact.point.z)) {
            continue;
        }
        
        // Create and add contact point
        corgi_msgs::ContactPoint cp;
        cp.position = contact.point;
        cp.force_magnitude = 0.0;
        
        structured_data.legs[leg_idx].rims[rim_idx].contact_points.push_back(cp);
        structured_data.legs[leg_idx].rims[rim_idx].has_contact = true;
        structured_data.legs[leg_idx].has_contact = true;
        
        total_points++;
    }
    
    structured_data.total_contact_points = total_points;
    return structured_data;
}

bool ContactProcessor::hasLegContact(const corgi_msgs::StructuredContactDataStamped& data, const std::string& leg_name) {
    auto it = std::find(leg_names.begin(), leg_names.end(), leg_name);
    if (it != leg_names.end()) {
        int idx = std::distance(leg_names.begin(), it);
        return data.legs[idx].has_contact;
    }
    return false;
}

bool ContactProcessor::hasRimContact(const corgi_msgs::StructuredContactDataStamped& data, const std::string& leg_name, const std::string& rim_name) {
    auto leg_it = std::find(leg_names.begin(), leg_names.end(), leg_name);
    auto rim_it = std::find(rim_names.begin(), rim_names.end(), rim_name);
    
    if (leg_it != leg_names.end() && rim_it != rim_names.end()) {
        int leg_idx = std::distance(leg_names.begin(), leg_it);
        int rim_idx = std::distance(rim_names.begin(), rim_it);
        return data.legs[leg_idx].rims[rim_idx].has_contact;
    }
    return false;
}

std::vector<geometry_msgs::Point> ContactProcessor::getRimContactPoints(const corgi_msgs::StructuredContactDataStamped& data, 
                                                      const std::string& leg_name, 
                                                      const std::string& rim_name) {
    std::vector<geometry_msgs::Point> points;
    auto leg_it = std::find(leg_names.begin(), leg_names.end(), leg_name);
    auto rim_it = std::find(rim_names.begin(), rim_names.end(), rim_name);
    
    if (leg_it != leg_names.end() && rim_it != rim_names.end()) {
        int leg_idx = std::distance(leg_names.begin(), leg_it);
        int rim_idx = std::distance(rim_names.begin(), rim_it);
        
        for (const auto& cp : data.legs[leg_idx].rims[rim_idx].contact_points) {
            points.push_back(cp.position);
        }
    }
    
    return points;
}