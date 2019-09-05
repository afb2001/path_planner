#include <gtest/gtest.h>
#include "NodeStub.h"
#include "../../src/executive/executive.h"

using std::vector;
using std::pair;
using std::cerr;
using std::endl;

TEST(SystemTests, LoadMapTest) {
    std::stringstream buffer;
    std::streambuf* sbuf = std::cerr.rdbuf();
    std::cerr.rdbuf(buffer.rdbuf());
    NodeStub stub;
    auto executive = new Executive(&stub);
    executive->addToCover(20, 20);
    executive->updateCovered(0, 0, 2.3, M_PI / 4, Executive::getCurrentTime());
    executive->startPlanner("");
    executive->refreshMap("/home/abrown/Downloads/depth_map/US5NH02M.tiff");
    for (int i = 0; i <= 25; i++) {
        auto found = buffer.str().find("Done loading map") != -1;
        EXPECT_TRUE(found || i < 25);
        if (found) break;
        ASSERT_FALSE(buffer.str().find("Encountered an error loading map at path ") != -1);
        sleep(1);
        executive->updateCovered(1.6263455967290593 * (i+1), 1.6263455967290593 * (i+1), M_PI / 4, 2.3, Executive::getCurrentTime());
    }
    executive->pause();
    std::cerr.rdbuf(sbuf);
    std::cerr << buffer.str() << endl;
    delete executive;
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}