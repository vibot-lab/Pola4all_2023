#ifndef __LOAD_SAVE_EXPERIMENTS_HPP__
#define __LOAD_SAVE_EXPERIMENTS_HPP__

// OpenCV includes
#include "opencv2/core/core.hpp"

// STD includes
#include <string>
#include <vector>

// Qt Includes

// Pylon includes

// Custom includes
#include "CameraTypes.hpp"

// ROS includes

/**
 * LoadSaveExperiments class: This class helps save an experiment, in which
 * several images are taken. The experiment is to provide a fixed linearly polarized
 * light to the camera, and to take several samples of it. This will allow to make an
 * average pixel-wise to reduce the influence of the Gaussian noise. Then, if
 * we take samples for all the possible angles, we will have, per pixel, a cosine
 * function evolution.
 *
 * This class allows to make the average per angle, store the images into disk,
 * and later on, load them into memory again. This way, we do not have
 * to repeat the experiment, and we can use always the dataset.
 *
 * An experiment is stored in a single folder. The folder name will be the experiment
 * name and the timestamp of when it was started.
 *
 * Inside this folder, we will create several files (one per sample), and per angle.
 * The naming convention is:
 *                              angle_N_sample_M.png
 * where N is the angle of the linearly polarized light source, and M is the sample
 * number.
 *
 * Finally, a function is included to list all the folders in a given path.
 * The idea is that we can have several experiments in one folder, and we can
 * choose the one we want. For that, a list of the present folders is provided.
*/
class LoadSaveExperiments
{
public:
    /**
     * @brief Constructor
     *
     * @arg camImgFormat: Struct with the camera image bit-depth format information.
    */
    LoadSaveExperiments(std::shared_ptr<bitDepthStruct> camImgFormat);
    /// \brief Destructor
    ~LoadSaveExperiments();

    /**
     * @brief findExperimentsList: Get a list of all the folders present in a
     * given directory. Each folder is considered an experiment, so this function
     * provides the list of experiments availables in the provided directory.
     *
     * @arg rootPath: Path where to check the existence of directories.
     *
     * @returns Vector of strings. Each string provides the full path of the found
     * directories.
    */
    std::vector<std::string> findExperimentsList(std::string rootPath);

    /**
     * @brief startExperiment: Initialize the directories and the internal variables
     * to start a new experiment. This function will try to create the destination
     * directory, which will have the experiment name, and the directory creation
     * timestamp.
     *
     * @arg storePath: String with the path where to store the experiment.
     * @arg experimentName: String with the desired experiment's name.
     *
     * @returns: Boolean. True if the experiment directory was created successfully. False otherwise.
    */
    bool startExperiment(std::string storePath, std::string experimentName);

    /**
     * @brief addExperimentSample: Add a sample to the experiment. If the angleOfPolarization
     * is the same as in the previous sample, we will add the given image to the
     * global average, if not, we will create a new entry, and start a new average.
     *
     * @arg img: Image to add to the average.
     * @arg angleOfPolarization: Angle, measured in degrees, that corresponds to the
     * light source linear polarization.
     * @arg storeImage: If true, the input image will be written in disk.
    */
    void addExperimentSample(cv::Mat img, int angleOfPolarization, bool storeImage);

    /**
     * @brief loadExperiment: Read the images stored in an experiment folder, and
     * take them as input images. Then, all the samples of the same polarization angle
     * will be averaged, and stored in memory in this class.
     *
     * @arg rootPath: Root path, where the experiment is located.
     * @arg expName: Experiment name (directory name) to be loaded.
    */
    void loadExperiment(std::string rootPath, std::string expName);

    /**
     * @brief endExperiment: Function to be called, each time the load / new experiment
     * process finishes. Since the values in the vector might not be sorted,
     * we sort the elements based on the angle of polarization of the light.
     * The entries are sorted in increasing angle order.
    */
    void endExperiment();

    /**
     * @brief getImages: Get an image with the sample taken from the experiment.
     *  Each channel of this image contains the average of all the images taken at the
     * same polarization orientation.
     *
     * @returns: Multi-channel cv::Mat objects. Since the data to return is huge,
     *  we give only a read-only reference to avoid copying the data.
    */
    const cv::Mat& getImages() const {return _sampleAverageImg;}

    /**
     * @brief getAngles: Get the vector with the sample angles from the experiment.
     * Each element contains the polarization angle of the input light used to
     * create that sample.
     *
     * @returns: Vector of doubles, with the orientation angles of the source light.
    */
    const std::vector<double>& getAngles() const {return _sampleAngles;}

    /// \brief reset: Clear the internal samples vectors.
    void reset();

    /// \brief getLastExperimentPath: Get the directory path of the last loaded experiment
    std::string getLastExperimentPath() const {return _storagePath;}

    /// \brief getAmountSamples: Get the amount of loaded samples.
    int getAmountSamples() const {return _sampleAverageImg.channels();}

private:
    /**
     * @brief splitString: Split a string into substrings. The split point is the
     * given separator character. The separator character is not present in the output
     * strings.
     *
     * @arg filename: Input string
     * @arg separator: Separator character
     *
     * @returns Vector of strings. If the input string is a filepath, for instance,
     * and the separator character is "/", the output vector will contains as many
     * entries as directories are present the in filepath tree, plus the filename.
    */
    std::vector<std::string> splitString(std::string filename, char separator);

    /**
     * @brief getListOfFiles: Get a list of all the files present in a folder.
     * This function is not recursive, so it does not explore the folders inside
     * the provided path, and it ignores the folders.
     *
     * @arg rootPath: String with the absolute path to explore.
     *
     * @returns Vector of strings. Each element in this vector contains the absolute
     * path of a file present in the folder provided.
    */
    std::vector<std::string> getListOfFiles(std::string rootPath);

    std::string _storagePath;
    std::vector<experimentSamples> _samples;
    std::vector<cv::Mat> _vectorOfSamplesAverage;
    cv::Mat _sampleAverageImg;
    std::vector<double> _sampleAngles;
    std::shared_ptr<bitDepthStruct> _camImgFormat;
    int _lastLoadedDatatype;
};

#endif // __LOAD_SAVE_EXPERIMENTS_HPP__