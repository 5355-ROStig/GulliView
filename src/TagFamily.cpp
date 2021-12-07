#include "TagFamily.h"

familyMapping TagFamily::getFamilies() {
    static std::map<std::string, std::pair<apriltag_family_t *(*)(), void (*)(apriltag_family_t *)>> mapping;

    if (mapping.empty()) {
        mapping.insert({"tag36h11", std::make_pair(tag36h11_create, tag36h11_destroy)});
        mapping.insert({"tag25h9", std::make_pair(tag25h9_create, tag25h9_destroy)});
        mapping.insert({"tag16h5", std::make_pair(tag16h5_create, tag16h5_destroy)});
        mapping.insert({"tagCircle21h7", std::make_pair(tagCircle21h7_create, tagCircle21h7_destroy)});
        mapping.insert({"tagCircle49h12", std::make_pair(tagCircle49h12_create, tagCircle49h12_destroy)});
        mapping.insert({"tagStandard41h12", std::make_pair(tagStandard41h12_create, tagStandard41h12_destroy)});
        mapping.insert({"tagStandard52h13", std::make_pair(tagStandard52h13_create, tagStandard52h13_destroy)});
        mapping.insert({"tagCustom48h12", std::make_pair(tagCustom48h12_create, tagCustom48h12_destroy)});
    }

    return mapping;
}

std::vector<std::string> TagFamily::getFamilyNames() {
    std::vector<std::string> vec;
    for (auto& p : getFamilies()) {
        vec.push_back(p.first);
    }

    return vec;
}

TagFamily::TagFamily(std::string &name) {
    if(getFamilies().count(name) <= 0) {
        std::cout << "Invalid tag name" << std::endl;
        throw std::runtime_error("Invalid ");
    }
    auto pair = getFamilies().at(name);
    at_family = pair.first();
    m_destructor = pair.second;
}

TagFamily::~TagFamily() {
    m_destructor(at_family);
}
