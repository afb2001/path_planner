#ifndef SRC_DYNAMICOBSTACLESMANAGERBASE_H
#define SRC_DYNAMICOBSTACLESMANAGERBASE_H

#include <unordered_map>
#include <unordered_set>
#include "DynamicObstaclesManager.h"

class DynamicObstaclesManagerBase : public DynamicObstaclesManager {
public:
    /**
     * Ignore a specific dynamic obstacle. This is intended to be used to ignore a parent vessel to the ASV.
     * @param mmsi
     */
    void addIgnore(uint32_t mmsi) {
        m_Ignored.emplace(mmsi);
    };

    /**
     * Stop ignoring a specific dynamic obstacle.
     * @param mmsi
     */
    void removeIgnore(uint32_t mmsi){
        m_Ignored.erase(mmsi);
    };

protected:
    bool isIgnored(uint32_t mmsi) { return m_Ignored.find(mmsi) != m_Ignored.end(); }

private:
    std::unordered_set<uint32_t> m_Ignored;
};

#endif //SRC_DYNAMICOBSTACLESMANAGERBASE_H
