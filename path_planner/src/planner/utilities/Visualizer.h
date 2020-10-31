#ifndef SRC_VISUALIZER_H
#define SRC_VISUALIZER_H

#include <fstream>
#include <memory>

/**
 * Encapsulate IO for visualization.
 */
class Visualizer {
public:
    typedef std::shared_ptr<Visualizer> SharedPtr;
    typedef std::unique_ptr<Visualizer> UniquePtr;
    explicit Visualizer(const std::string& path) {
        m_Stream.open(path, std::ios::trunc | std::ios::out);
    }

    std::ofstream& stream() {
        return m_Stream;
    }

private:
    std::ofstream m_Stream;
};


#endif //SRC_VISUALIZER_H
