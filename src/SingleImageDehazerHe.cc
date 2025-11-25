//
// Created by tedyiningding on 24/04/23.
//

#include "SingleImageDehazerHe.h"

namespace ORB_SLAM2
{

SingleImageDehazerHe::SingleImageDehazerHe(const cv::Mat &hazy_img, eAtmosMode atmos_mode): ORB_SLAM2::SingleImageDehazerAbstract(hazy_img)
{
    mAtmosMode = atmos_mode;
};

void SingleImageDehazerHe::CalcDarkChannel(int patch_size)
{
    // min operation per patch
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(patch_size, patch_size));
    cv::Mat eroded_img;
    cv::erode(mHazyImg, eroded_img, kernel);

    // min operation along the channel dimension (does nothing if the input hazy image is gray)
    cv::reduce(eroded_img.reshape(1, eroded_img.total()), mDarkChannelImg, 1, cv::REDUCE_MIN);
    mDarkChannelImg = mDarkChannelImg.reshape(0, eroded_img.rows);
}

void SingleImageDehazerHe::CalcAtmosLight()
{
    const float fraction = 0.001f;
    const int num_search_pixels = floor(fraction * mHazyImg.total());

    assert(mDarkChannelImg.isContinuous()
            && "the dark channel image has to be continuous in memory for fast conversion to a vector");

    // convert dark image intensities to a vector
    std::vector<uchar> vec_dark_intensities;
    vec_dark_intensities.assign(mDarkChannelImg.data, mDarkChannelImg.data + mDarkChannelImg.total());
    // build a vector to store the indices 0, 1, 2, ...
    std::vector<int> vec_dark_indices(vec_dark_intensities.size());
    std::iota(std::begin(vec_dark_indices), std::end(vec_dark_indices), 0);
    // partially sort the indices according to the top 0.1% highest intensities in descending order
    std::partial_sort(vec_dark_indices.begin(),
                      vec_dark_indices.begin() + num_search_pixels,
                      vec_dark_indices.end(),
                      [&vec_dark_intensities](size_t i1, size_t i2) {return vec_dark_intensities[i1] > vec_dark_intensities[i2];});
    
    // methods to find the atmospheric light
    switch (mAtmosMode)
    {
        case MAX_INTENSITY:
        {
            if (mnChs == 3)
            {
                // convert the hazy image from colour to gray
                cv::Mat hazy_gray_img_from_colour;
                cv::cvtColor(mHazyImg, hazy_gray_img_from_colour, cv::COLOR_BGR2GRAY);

                // iterate these pixel locations in the gray hazy image and find the location with the highest intensity
                // the atmospheric light is selected to be the intensity at this location in the hazy image
                uchar highest_intensity = 0;    // to store the highest intensity
                cv::Vec3b atmos_u8;             // to store the candidate atmospheric light intensity
                for (int i = 0; i < num_search_pixels; i++)
                {
                    int ind = vec_dark_indices[i];
                    int v = ind / mnCols;
                    int u = ind % mnCols;
                    uchar current_intensity = hazy_gray_img_from_colour.at<uchar>(v, u);
                    if (current_intensity > highest_intensity)
                    {
                        highest_intensity = current_intensity;
                        atmos_u8 = mHazyImg.at<cv::Vec3b>(v, u);
                    }
                }
                mAtmosLight[0] = float(atmos_u8[0]);
                mAtmosLight[1] = float(atmos_u8[1]);
                mAtmosLight[2] = float(atmos_u8[2]);
            }
            else
            {
                // iterate these pixel locations in the gray hazy image and find the location with the highest intensity
                // the atmospheric light is selected to be the intensity at this location in the hazy image
                uchar highest_intensity = 0;    // to store the candidate atmospheric light intensity
                for (int i = 0; i < num_search_pixels; i++)
                {
                    int ind = vec_dark_indices[i];
                    int v = ind / mnCols;
                    int u = ind % mnCols;
                    uchar current_intensity = mHazyImg.at<uchar>(v, u);
                    if (current_intensity > highest_intensity)
                    {
                        highest_intensity = current_intensity;
                    }
                }
                mAtmosLight[0] = float(highest_intensity);
                mAtmosLight[1] = float(highest_intensity);
                mAtmosLight[2] = float(highest_intensity);
            }

            break;
        }

        case MEDIAN_INTENSITY:
        {
            // get vector of Pixel which stores the most haze-opaque pixels and their locations
            std::vector<Pixel> vec_most_hazy_pixels;
            vec_most_hazy_pixels.reserve(num_search_pixels);
            if (mnChs == 3)
            {
                // convert the hazy image from colour to gray
                cv::Mat hazy_gray_img_from_colour;
                cv::cvtColor(mHazyImg, hazy_gray_img_from_colour, cv::COLOR_BGR2GRAY);

                for (int i = 0; i < num_search_pixels; i++)
                {
                    int ind = vec_dark_indices[i];
                    int v = ind / mnCols;
                    int u = ind % mnCols;
                    uchar intensity = hazy_gray_img_from_colour.at<uchar>(v, u);
                    vec_most_hazy_pixels.emplace_back(intensity, u, v);
                }
            }
            else
            {
                for (int i = 0; i < num_search_pixels; i++)
                {
                    int ind = vec_dark_indices[i];
                    int v = ind / mnCols;
                    int u = ind % mnCols;
                    uchar intensity = mHazyImg.at<uchar>(v, u);
                    vec_most_hazy_pixels.emplace_back(intensity, u, v);
                }
            }

            // find the u and v at which the median intensity occurs
            auto m = vec_most_hazy_pixels.begin() + vec_most_hazy_pixels.size()/2;
            std::nth_element(vec_most_hazy_pixels.begin(), m, vec_most_hazy_pixels.end(), std::greater<Pixel>());
            int v = vec_most_hazy_pixels[vec_most_hazy_pixels.size() / 2].m_v;
            int u = vec_most_hazy_pixels[vec_most_hazy_pixels.size() / 2].m_u;

            if (mnChs == 3)
            {
                cv::Vec3b atmos_u8 = mHazyImg.at<cv::Vec3b>(v, u);
                mAtmosLight[0] = float(atmos_u8[0]);
                mAtmosLight[1] = float(atmos_u8[1]);
                mAtmosLight[2] = float(atmos_u8[2]);
            }
            else
            {
                uchar atmos_u8 = mHazyImg.at<uchar>(v, u);
                mAtmosLight[0] = float(atmos_u8);
                mAtmosLight[1] = float(atmos_u8);
                mAtmosLight[2] = float(atmos_u8);
            }

            break;
        }

        case MEAN_INTENSITY:
        {
            // accumulate intensities then average
            if (mnChs == 3)
            {
                cv::Vec3f accumulator(0.0,0.0,0.0);
                for (int i = 0; i < num_search_pixels; i++)
                {
                    int ind = vec_dark_indices[i];
                    int v = ind / mnCols;
                    int u = ind % mnCols;
                    cv::Vec3b intensity = mHazyImg.at<cv::Vec3b>(v, u);
                    accumulator += intensity * 1.0f;
                }
                mAtmosLight = accumulator / num_search_pixels;
            }
            else
            {
                float accumulator = 0.0;
                for (int i = 0; i < num_search_pixels; i++)
                {
                    int ind = vec_dark_indices[i];
                    int v = ind / mnCols;
                    int u = ind % mnCols;
                    uchar intensity = mHazyImg.at<uchar>(v, u);
                    accumulator += float(intensity);
                }
                float average = accumulator / num_search_pixels;
                mAtmosLight[0] = average;
                mAtmosLight[1] = average;
                mAtmosLight[2] = average;
            }

            break;
        }
    }
}

}