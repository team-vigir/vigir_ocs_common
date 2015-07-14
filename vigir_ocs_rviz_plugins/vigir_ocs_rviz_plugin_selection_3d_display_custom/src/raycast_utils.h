/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
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
@TODO_ADD_AUTHOR_INFO
/*
 * RayCastUtils class declaration.
 *
 * Author: Felipe Bacim.
 *
 * Based on code from http://www.ogre3d.org/tikiwiki/Raycasting+to+the+polygon+level
 * and http://komando-game.googlecode.com/svn-history/r22/OgreGame/SimpleCollision/
 *
 * Latest changes (12/11/2012):
  */

#ifndef _RAYCAST_UTILS_H_
#define _RAYCAST_UTILS_H_

#include <OGRE/OgrePrerequisites.h>
#include <OGRE/OgreSceneQuery.h>
#include <OGRE/OgreSceneManager.h>
#include <point_cloud_custom.h>

class RayCastUtils
{
public:
    RayCastUtils(Ogre::SceneManager* sm);
    ~RayCastUtils();

    bool RayCastFromPoint(const Ogre::Vector3 &point, const Ogre::Vector3 &normal, Ogre::Vector3 frame_pos, Ogre::Quaternion frame_quat, Ogre::Vector3 &result, int &object_type, std::string &object_name, bool prioritize=true);
    bool RayCastFromPoint(const Ogre::Ray ray, Ogre::Vector3 frame_pos, Ogre::Quaternion frame_quat, Ogre::Vector3 &result, int &object_type, std::string &object_name, bool prioritize=true);
    
    void GetMeshInformationPointCloud(rviz::PointCloudCustom* &mesh, std::vector<Ogre::Vector3> &vertices, std::vector<unsigned long> &indices,
                                      Ogre::Vector3 frame_pos, Ogre::Quaternion frame_quat);

    void GetMeshInformationOptimized(const Ogre::MeshPtr mesh, size_t &vertex_count, Ogre::Vector3* &vertices, size_t &index_count,
        unsigned long* &indices, const Ogre::Vector3 &position,	const Ogre::Quaternion &orient,	const Ogre::Vector3 &scale);

    void GetMeshInformationForAnimations(const Ogre::Entity *entity, size_t &vertex_count, Ogre::Vector3* &vertices, size_t &index_count,
        unsigned long* &indices, const Ogre::Vector3 &position,	const Ogre::Quaternion &orient,	const Ogre::Vector3 &scale);

    void GetMeshInformation64bit(Ogre::Entity *entity, size_t &vertex_count, Ogre::Vector3* &vertices, size_t &index_count, Ogre::uint32* &indices,
        const Ogre::Vector3 &position, const Ogre::Quaternion &orient, const Ogre::Vector3 &scale);
private:
    Ogre::RaySceneQuery* ray_scene_query_;
    Ogre::SceneManager* scene_manager_;

};


#endif
