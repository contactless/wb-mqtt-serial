#include <gtest/gtest.h>
#include <wblib/testing/testlog.h>
#include <wblib/utils.h>

using WBMQTT::Testing::TLoggedFixture;

int main(int argc, char **argv)
{
    WBMQTT::SetThreadName("main");
    TLoggedFixture::SetExecutableName(argv[0]);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
