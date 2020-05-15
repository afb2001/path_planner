#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cert-err58-cpp"
#include <gtest/gtest.h>
#include <thread>
#include "NodeStub.h"
#include "../../src/executive/executive.h"

using std::vector;
using std::pair;
using std::cerr;
using std::endl;

TEST(SystemTests, LoadMapTest) {
    auto startTime = Executive::getCurrentTime();
    std::stringstream buffer;
    std::streambuf* sbuf = std::cerr.rdbuf();
    std::cerr.rdbuf(buffer.rdbuf());
    NodeStub stub;
    auto executive = new Executive(&stub);
    executive->addRibbon(20, 20, 20, 30);
    executive->updateCovered(0, 0, 0, M_PI / 4, Executive::getCurrentTime());
    executive->refreshMap("/home/abrown/Downloads/depth_map/US5NH02M.tiff", 43.073397415457535, -70.71054174878898);
    for (int i = 0; i <= 60; i++) {
        auto found = buffer.str().find("Done loading map") != -1;
        EXPECT_TRUE(found || i < 60);
        if (found) break;
        ASSERT_FALSE(buffer.str().find("Encountered an error loading map at path ") != -1);
        sleep(1);
        executive->updateCovered(1.6263455967290593 * (i+1), 1.6263455967290593 * (i+1), M_PI / 4, 2.3, Executive::getCurrentTime());
    }
    executive->cancelPlanner();
    std::cerr.rdbuf(sbuf);
    std::cerr << buffer.str() << endl;
    cerr << "Total time elapsed: " << Executive::getCurrentTime() - startTime << "s" << endl;
    delete executive;
}

TEST(SystemTests, SimpleTwoLineTest) {
    NodeStub stub;
    auto executive = new Executive(&stub);
    executive->addRibbon(10, 10, 20, 10);
    executive->addRibbon(10, 20, 20, 20);
    executive->updateCovered(0, 0, 0, 0, Executive::getCurrentTime());
    executive->startPlanner();
    for (int i = 0; i < 120; i++) {
        if (stub.allDoneCalled()) break;
        if (!stub.lastTrajectory().empty()) {
            auto start = stub.lastTrajectory()[1];
            cerr << start.toString() << endl;
            executive->updateCovered(start.x(), start.y(), start.speed(), start.heading(),
                                     i + 1); // maybe use start's time here?
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    EXPECT_TRUE(stub.allDoneCalled());
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
#pragma clang diagnostic pop