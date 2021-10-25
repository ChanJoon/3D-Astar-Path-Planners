#ifndef SAVEDATAVARIANTTOFILE_HPP
#define SAVEDATAVARIANTTOFILE_HPP

/**
 * @file DataVariantToFile.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com )
 * @brief  Save std::map<std::string, custom_varian> to file
 * @version 0.1
 * @date 2021-06-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include <fstream>
#include <algorithm>
#include <numeric>
#include "utils/utils.hpp"

/* Test for GCC > 9.0.0 */
#if __GNUC__ >= 9 
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif
namespace Planners
{
    namespace utils
    {
        /**
         * @brief Deduction guide 
         * 
         * @tparam Ts 
         */
        template <class... Ts>
        struct overloaded : Ts...
        {
            using Ts::operator()...;
        };

        template <class... Ts>
        overloaded(Ts...) -> overloaded<Ts...>;
        
        /**
         * @brief This class allow to easily save information stored as "PathData" (std::map<std::string, DataVariant>) into files 
         * 
         */
        class DataVariantSaver
        {

        public:

            /**
             * @brief Construct a new Data Variant Saver object
             * 
             * @param _data_file data file path 
             * @param _fields A std::vector<std::string> of fields to look for on the input map. The default values 
             * are: algorithm, goal_coords, start_coords, time_spent, explored_nodes, path_length, line_of_sight_checks and solved
             */
            DataVariantSaver(const std::string &_data_file, 
                             const std::vector<std::string> &_fields =
                            {"algorithm", "goal_coords", "start_coords", "time_spent",
                             "explored_nodes", "path_length", "total_cost", "h_cost", "g_cost", "c_cost", "grid_cost", "g_final_node",
                             "line_of_sight_checks", "min_dist", "max_dist", "mean_dist", "std_dev",
                             "solved", "cost_weight","max_line_of_sight_cells", "av_curv", "std_dev_curv", "min_curv", "max_curv" }): fields_(_fields), data_file_(_data_file)
            {

            }
            /**
             * @brief Main function that reads the incoming pathdata object and save to file in append mode
             * 
             * @param _data pathdata input object 
             * @return true Always returns true at this version
             * @return false Never returns false right now
             */
            bool savePathDataToFile(const PathData &_data)
            {
                if( ! fs::exists(data_file_) ){ //If file does not exist, write a header with field names
                    std::cout << "File does not exists. Creating header at first line" << std::endl;   
                    out_file_data_.open(data_file_, std::ofstream::app);
                    out_file_data_ << fields_ << std::endl;
                }else{
                    out_file_data_.open(data_file_, std::ofstream::app);
                }
                for (auto &it : fields_)
                {
                    auto field = _data.find(it);
                    if (field == _data.end())
                    {
                        out_file_data_ << " , ";
                        continue;
                    }

                    std::visit(overloaded{
                            [this](auto arg) { out_file_data_ << arg << ", "; }, },
                            field->second);
                }
                out_file_data_ << std::endl;
                out_file_data_.close();


                return true;
            }
            bool savePathDistancesToFile(const utils::CoordinateList &_path,
                                         const std::vector<std::pair<utils::Vec3i, double>> &_results){
                
                out_file_data_.open(data_file_, std::ofstream::app);

                if( _path.size() != _results.size() )
                    return false;
                
                for(size_t i = 0; i < _results.size() -1 ; ++i)
                    out_file_data_ << _results[i].second << ", ";

                out_file_data_ << _results[static_cast<size_t>(_results.size() -1)].second << std::endl;;

                out_file_data_.close();

                return true;
            }

        private:
            std::ofstream out_file_data_;
            std::vector<std::string> fields_;
            std::string data_file_;
        };
    } // utils ns
} //planners ns

#endif