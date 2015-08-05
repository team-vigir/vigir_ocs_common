/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO
#include "vigir_ocs_msgs/RobotStatusCodes.h"
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <iostream>
#include <sstream>


RobotStatusCodes::RobotStatusCodes()
{
    messages_.resize(MAX_ERROR_MESSAGES+1,"Default Error Message");
    messages_[ MAX_ERROR_MESSAGES] = "Invalid Error Code"; // Initialize to allow return of value
}

void RobotStatusCodes::loadErrorMessages(const std::string& filename)
{
    std::ifstream file(filename.c_str());
    std::string val;
    std::string line;
    int l = 1;
    while(file.good())
    {
        getline(file,line); // read entire line
        std::stringstream ss(line); // convert to stream
        
        // Get data in this line only to comma
        getline(ss,val,','); 
        boost::replace_all(val, " ", "");
        
        l++;
        //std::cout << "line " << l << " (" << line.length() << ", " << val.length() << "): " << line << std::endl;
        
        if(val.length() == 0 || val.at(0) == '#')
        {
            std::cout << "skipping comment: line(" << l << ")=<" << line << ">" << std::endl;
        }
        else
        {
            try
            {
                if(boost::lexical_cast<int>(val.c_str()) <=MAX_ERROR_MESSAGES && boost::lexical_cast<int>(val.c_str()) >= 0)
                {
                    int num = boost::lexical_cast<int>(val.c_str());
                    getline(ss,val); // get remainder of text in this line
                    if (val.length() > 0)
                    {
                        if (val[val.length()-1] == ',')
                        {// Strip any terminating commas
                            val[val.length()-1] = 0;
                        }
                        messages_[num] = val.substr(0,val.length());//-1);
                        std::cout << "Loading message(" << num << ") = <" << messages_[num] << ">" << std::endl;
                    }
                    else
                    {
                        std::cout << "Skipping invalid message at line(" << l << ") = <" << val << ">" << std::endl;
                        std::cout << "    line<" << line << ">" << std::endl;
                    }
                }
                else
                {
                    std::cout << "Skipping message at line(" << l << ") = <" << val << ">" << std::endl;
                }
            }
            catch(...)
            {
            }
        }
    }
}
