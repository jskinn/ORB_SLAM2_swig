/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2019 John Skinner <jskinn7 at gmail dot com>
* For more information see <https://github.com/jskinn/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include "VocabularyBuilder.h"
#include "Thirdparty/DBoW2/DUtils/Random.h"
#include "ORBVocabulary.h"


void changeStructure(const cv::Mat &plain, std::vector<cv::Mat> &out);

namespace ORB_SLAM2
{

    VocabularyBuilder::VocabularyBuilder(
        int nfeatures,
        float scaleFactor,
        int nlevels,
        int edgeThreshold,
        int firstLevel,
        int WTA_K,
        int scoreType,
        int patchSize,
        int fastThreshold
    ) : orb(cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold))
    {

    }

    void VocabularyBuilder::addImage(const cv::Mat& image)
    {
        // Extract ORB features from the image, and add it to our build-up of featueres to construct the 
        cv::Mat mask;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        this->orb->detectAndCompute(image, mask, keypoints, descriptors);
        this->features.push_back(std::vector<cv::Mat>());
        changeStructure(descriptors, this->features.back());
    }

    void VocabularyBuilder::reset()
    {
        // Clear all accumulated ORB feature descriptors
        this->features.clear();
    }

    int VocabularyBuilder::getNumFeatures() const
    {
        int num_images = 0;
        for (auto iter = this->features.begin(); iter != this->features.end(); ++iter) {
            num_images += iter->size();
        }
        return num_images;
    }

    void VocabularyBuilder::buildVocabulary(const std::string &strVocFile, int branchingFactor, int numLevels, int seed)
    {
        // Build the vocab and save it to file.
        // These are hard-coded to be the same as the default ORBSLAM2 vocab.
        const DBoW2::WeightingType weight = DBoW2::TF_IDF;
        const DBoW2::ScoringType score = DBoW2::L1_NORM;

        DUtils::Random::SeedRandOnce(seed);  // Seed the RNG to control the K-means
        DUtils::Random::SeedRand(seed);      // Really actually set the seed to a particular value, even if already seeded.
        ORB_SLAM2::ORBVocabulary voc(branchingFactor, numLevels, weight, score);
        // Create the vocabulary
        voc.create(this->features);
        // Save it to the specified file
        voc.saveToTextFile(strVocFile);
        // Delete the features.
        // They've been changed by the clutering step of voc.create, and I'm not sure how to prevent this.
        this->reset();
    }
}


void changeStructure(const cv::Mat &plain, std::vector<cv::Mat> &out)
{
    /**
     * Helper to split a single matrix into a vector of row matricies.
     * Copied from the DBoW2 demo example. See:
     * https://github.com/dorian3d/DBoW2/blob/master/demo/demo.cpp#L90
     */
    out.resize(plain.rows);
    for(int i = 0; i < plain.rows; ++i)
    {
        out[i] = plain.row(i);
    }
}
