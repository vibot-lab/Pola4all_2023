// OpenCV includes
#include <opencv2/highgui.hpp>

// STD includes
#include <algorithm>
#include <experimental/filesystem>
#include <iostream>

// Qt Includes

// Pylon includes

// Custom includes
#include "PolarimetricCamera/LoadSaveExperiments/LoadSaveExperiments.hpp"

// ROS includes

namespace fs = std::experimental::filesystem;
static std::string dirSeparator = "/";

LoadSaveExperiments::LoadSaveExperiments(std::shared_ptr<bitDepthStruct> camImgFormat) :
    _storagePath(""),
    _camImgFormat(camImgFormat),
    _lastLoadedDatatype(-1)
{}

LoadSaveExperiments::~LoadSaveExperiments()
{
    reset();
}

void LoadSaveExperiments::reset()
{
    _samples.clear();
    _vectorOfSamplesAverage.clear();
    _sampleAverageImg = cv::Mat();
    _lastLoadedDatatype = -1;
}

std::vector<std::string> LoadSaveExperiments::findExperimentsList(std::string rootPath)
{
    std::vector<std::string> dirsFound;
    for(auto& p : fs::directory_iterator(rootPath))
    {
        // We only consider directories
        if (fs::is_directory(p))
        {
            std::string path(p.path());
            // We see which is the last directory separator. The string that
            // follows corresponds to the folder name
            int idx = path.find_last_of(dirSeparator);
            // We take the next character to the provided index to avoid the separator
            dirsFound.push_back(path.substr(idx + 1, path.length()));
        }
    }

    return dirsFound;
}

bool LoadSaveExperiments::startExperiment(std::string storePath, std::string experimentName)
{
    /// If the directory does not exists, we create it.
    if (!fs::exists(storePath))
    {
        std::cout << "Directory " << storePath << " does not exists. Creating it..." << std::endl;
        fs::create_directories(storePath);
    }
    else
    {
        std::cout << "The directory " << storePath << " exists" << std::endl;
    }

    // We create the final experiment folder path
    std::string expPath = storePath + dirSeparator + experimentName;
    if (fs::exists(expPath))
    {
        std::cout << "ERROR: This experiment name is already in use. Choose another one. Aborting" << std::endl;
        return false;
    }

    /// If we fail to create the experiment directory, that means we cannot write
    // there, so we quit.
    if (!fs::create_directories(expPath))
    {
        std::cout << "ERROR cannot create the directory in the desired folder " << storePath << std::endl;
        return false;
    }

    // We set some internal variables, that we will use later on.
    fs::path dstPath = expPath;
    _storagePath = dstPath.u8string();
    reset();

    std::cout << "Experiment created at: " << expPath << std::endl;
    return true;
}

void LoadSaveExperiments::addExperimentSample(cv::Mat img, int angleOfPolarization, bool storeImage)
{
    float scalingFactor = 1.0;
    cv::Mat floatImg;
    _lastLoadedDatatype = img.type() & CV_MAT_DEPTH_MASK;
    if (_camImgFormat->_opencvType != _lastLoadedDatatype)
    {
        // Based on the bit-depth, we determine the maximum allowed value for
        // that datatype.
        float depthMaxVal = -1;
        switch(_lastLoadedDatatype)
        {
            case CV_8U:
            {
                depthMaxVal = 255.0;
                break;
            }
            case CV_16U:
            {
                depthMaxVal = 4095.0;
                break;
            }
            default:
            {
                assert(0);
                break;
            }
        }
        scalingFactor = _camImgFormat->_maxAllowedVal / depthMaxVal;
    }
    img.convertTo(floatImg, CV_64FC1, scalingFactor);

    /// We check if the samples vector is not empty, or if we change the angle of polarization.
    // If it is not the case, we add the image to the global average.
    if (_samples.size() && _samples.back().anglePol == angleOfPolarization)
    {
        _samples.back().averageImage = ((_samples.back().averageImage * _samples.back().N) + floatImg) / (_samples.back().N + 1);
        _samples.back().N += 1;
    }
    else
    {
        /// If we change the angle of polarization, or it is the first sample,
        // we initialize the average
        experimentSamples newObj;
        _samples.push_back(newObj);

        _samples.back().N = 1;
        _samples.back().anglePol = angleOfPolarization;
        floatImg.copyTo(_samples.back().averageImage);
    }

    /// We store the image into disk, if storeImage is true
    if (storeImage)
    {
        std::string filename = std::string("angle_") + std::to_string(angleOfPolarization) + "_sample_" + std::to_string(_samples.back().N) + ".png";
        fs::path filepath = _storagePath + dirSeparator + filename;
        cv::imwrite(filepath.u8string(), img);
    }
}

std::vector<std::string> LoadSaveExperiments::splitString(std::string filename, char separator)
{
    // Taken from https://stackoverflow.com/questions/5167625/splitting-a-c-stdstring-using-tokens-e-g
    std::vector<std::string> output;
    std::istringstream f(filename);
    std::string s;

    while (getline(f, s, separator)) {
        output.push_back(s);
    }

    return output;
}

std::vector<std::string> LoadSaveExperiments::getListOfFiles(std::string rootPath)
{
    std::vector<std::string> output;

    for(auto& p : fs::directory_iterator(rootPath))
    {
        // We only analyze the files, not the directories
        if (!fs::is_directory(p))
        {
            output.push_back(p.path());
        }
    }

    // We sort the array so all images with the same source light appears together.
    std::sort(output.begin(), output.end());
    return output;
}

// The files that we are going to look for has to have the format angle_N_sample_M.*
// where N and M are two integer numbers.
void LoadSaveExperiments::loadExperiment(std::string rootPath, std::string expName)
{
    _storagePath = std::string("");
    reset();

    if (rootPath[rootPath.length()] != dirSeparator[0])
    {
        rootPath += dirSeparator[0];
    }

    /// We check if the provided experiment folder exists
    std::string expPath = rootPath + expName;
    if (fs::exists(expPath))
    {
        _storagePath = expPath;
        // We check all the files present in the folder, and we go through them
        std::vector<std::string> filepaths = getListOfFiles(expPath);

        if (!filepaths.size())
        {
            std::cout << "This folder is empty. We are not loading anything" << std::endl;
        }

        for(auto& path : filepaths)
        {
            int idx = path.find_last_of(dirSeparator);

            // We extract the filename from the path
            std::string filename = path.substr(idx + 1, path.length());
            std::vector<std::string> splittedString = splitString(filename, '_');

            if (splittedString.size() == 4)
            {
                /// If the filename format is not correct, the stoi will fail with
                // invalid_argument exception
                try
                {
                    int angle = std::stoi(splittedString[1]);

                    /// If nothing went wrong until now, we can read the image
                    cv::Mat img = cv::imread(path.c_str(), cv::IMREAD_UNCHANGED);
                    // If we read the image successfuly, we add it to our average.
                    if (!img.empty())
                    {
                        addExperimentSample(img, angle, false);
                    }
                }
                catch(const std::invalid_argument& e)
                {
                    std::cerr << "WARNING: The input file cannot be converted into integer. Discarding file: " << filename << std::endl;
                    continue;
                }
            }
            else
            {
                std::cout << "WARNING: The splitted string has not the right amount of elements for the file: " << filename << std::endl;
            }
        }
        endExperiment();
    }
    else
    {
        std::cout << "ERROR: Could not load the experiment " << expName << " located at " << expPath << std::endl;
    }
}

void LoadSaveExperiments::endExperiment()
{
    if (_samples.size())
    {
        _sampleAngles.clear();
        _vectorOfSamplesAverage.clear();
        // Since the data to order is an struct, we use a lambda function to order them.
        // This lambda takes two experiment samples, and it returns a boolean.
        // If the experiment 1 is smaller than the experiment 2, we return True.
        std::sort(
            _samples.begin(),
            _samples.end(),
            [](experimentSamples e1, experimentSamples e2) {return (e1.anglePol < e2.anglePol);});

        std::transform(
            _samples.begin(),
            _samples.end(),
            std::back_inserter(_sampleAngles),
            [](const experimentSamples& v) {return v.anglePol;}
        );

        std::transform(
            _samples.begin(),
            _samples.end(),
            std::back_inserter(_vectorOfSamplesAverage),
            [](const experimentSamples& v) {return v.averageImage;}
        );

        // With this line, we convert the vector of images into a multi-channel image.
        cv::merge(_vectorOfSamplesAverage, _sampleAverageImg);
        // To reduce the storage space, we put back the original image
        // bit-depth. It should not affect too much the final precision.
        _sampleAverageImg.convertTo(_sampleAverageImg, _lastLoadedDatatype);

        // To save space, we erase the samples since we already store them in
        // the average img and the angles variables.
        _samples.clear();
        _vectorOfSamplesAverage.clear();
        std::cout << "Resulting images size: ("
                  << _sampleAverageImg.rows << ", "
                  << _sampleAverageImg.cols << ", "
                  << _sampleAverageImg.channels() << ")"<< std::endl;
    }
}