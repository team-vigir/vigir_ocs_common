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

    bool RayCastFromPoint(const Ogre::Vector3 &point, const Ogre::Vector3 &normal, Ogre::Vector3 frame_pos, Ogre::Quaternion frame_quat, Ogre::Vector3 &result);
    bool RayCastFromPoint(const Ogre::Ray ray, Ogre::Vector3 frame_pos, Ogre::Quaternion frame_quat, Ogre::Vector3 &result);
    
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
