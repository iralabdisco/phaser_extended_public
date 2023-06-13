#include "phaser/backend/uncertainty/neighbors-peak-extraction.h"

#include <gtest/gtest.h>

#include "phaser/common/test/testing-entrypoint.h"

namespace phaser_core {

    class NeighborsPeakTest : public ::testing::Test {
    protected:
        virtual void SetUp() {}
    };

    TEST(SanityCheckTest, SanityCheck) {
        EXPECT_TRUE(true);
    }

    TEST(MaxCheckTest, MaxTest) {
        NeighborsPeakExtraction peak_extractor(4, 1,
                                               3, false, true);

        std::set<uint32_t> peaks;
        std::vector<double> corr = {0, 2, 7, 1, 6, 0, 10, 3, 2, 11};
        std::vector<uint32_t> max_peaks;

        peaks.insert(2);
        peaks.insert(4);
        peaks.insert(6);
        peaks.insert(9);

        peak_extractor.getMaxPeaks(&peaks,
            &corr, &max_peaks);

        std::vector<uint32_t> expected_max_peaks = {9, 6, 2};
        ASSERT_EQ(max_peaks.size(), expected_max_peaks.size()) << "Vectors x and y are of unequal length";

        for (uint i = 0; i < max_peaks.size(); ++i) {
            EXPECT_EQ(max_peaks[i], expected_max_peaks[i]) << "Vectors x and y differ at index " << i;
        }

    }

}  // namespace phaser_core

MAPLAB_UNITTEST_ENTRYPOINT