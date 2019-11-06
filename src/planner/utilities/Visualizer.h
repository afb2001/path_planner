#ifndef SRC_VISUALIZER_H
#define SRC_VISUALIZER_H

#include <fstream>

class Visualizer {
public:
    typedef std::shared_ptr<Visualizer> SharedPtr;
    typedef std::unique_ptr<Visualizer> UniquePtr;
    explicit Visualizer(const std::string& path) {
        m_Stream.open(path, ios::app | ios::out);
    }

    ofstream& stream() {
        return m_Stream;
    }

private:
    ofstream m_Stream;
};


#endif //SRC_VISUALIZER_H
