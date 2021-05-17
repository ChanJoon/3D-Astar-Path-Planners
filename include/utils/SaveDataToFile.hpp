#ifndef SAVEDATATOFILE_HPP
#define SAVEDATATOFILE_HPP

#include <iostream>
#include <fstream>
#include "utils/utils.hpp"
#include <boost/algorithm/string.hpp>
namespace Planners
{
    namespace utils
    {
        class DataSaver
        {

        public:
            DataSaver(const std::string &_data_file)
            {
                out_file_data_.open(_data_file, std::ofstream::app);
            }
            bool savePathDataToFile(const PathData &_data)
            {
                std::string field;
                bool res = true;
                try
                {
                    field = "goal_coords";
                    if (_data.find(field) != _data.end())
                    {
                        auto goal_vec = std::any_cast<utils::Vec3i>(_data.at(field));
                        out_file_data_ << "[ " << goal_vec.x << ", " << goal_vec.y << ", " << goal_vec.z << " ]" << ", ";
                    }
                    else
                    {
                        out_file_data_ << ", ";
                        std::cerr << "Could not save field: " << field << " does not exist" << std::endl;
                    }
                    field = "start_coords";
                    if (_data.find(field) != _data.end())
                    {
                        auto start_vec = std::any_cast<utils::Vec3i>(_data.at(field));
                        out_file_data_ << "[ " << start_vec.x << ", " << start_vec.y << ", " << start_vec.z << " ]" << ", ";
                    }
                    else
                    {
                        out_file_data_ << ", ";
                        std::cerr << "Could not save field: " << field << " does not exist" << std::endl;
                    }
                    field = "time_spent";
                    if (_data.find(field) != _data.end())
                    {
                        auto time_spent = std::any_cast<double>(_data.at(field));
                        out_file_data_ << time_spent << ", ";
                    }
                    else
                    {
                        out_file_data_ << ", ";
                        std::cerr << "Could not save field: " << field << " does not exist" << std::endl;
                    }
                    field = "explored_nodes";
                    if (_data.find(field) != _data.end())
                    {
                        auto explored_nodes = std::any_cast<size_t>(_data.at(field));
                        out_file_data_ << explored_nodes << ", ";
                    }
                    else
                    {
                        out_file_data_ << ", ";
                        std::cerr << "Could not save field: " << field << " does not exist" << std::endl;
                    }
                    field = "path_length";
                    if (_data.find(field) != _data.end())
                    {
                        auto path_length = std::any_cast<float>(_data.at(field));
                        out_file_data_ << path_length << ", ";
                    }
                    else
                    {
                        out_file_data_ << ", ";
                        std::cerr << "Could not save field: " << field << " does not exist" << std::endl;
                    }
                    field = "line_of_sight_checks";
                    if (_data.find(field) != _data.end())
                    {
                        auto line_of_sight_checks = std::any_cast<int>(_data.at(field));
                        out_file_data_ << line_of_sight_checks << ", ";
                    }
                    else
                    {
                        out_file_data_ << ", ";
                        std::cerr << "Could not save field: " << field << " does not exist" << std::endl;
                    }
                    out_file_data_ << std::endl;
                }
                catch (const std::bad_any_cast &e)
                {
                    std::cerr << "Any cast error: " << e.what() << std::endl;
                    res = false;
                }

                out_file_data_.close();
                return res;
            }

        private:
            std::ofstream out_file_data_;
        };
    }
}

#endif