#include "raycast_utils.h"
#include <OGRE/OgreMovableObject.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreSubMesh.h>
#include <OGRE/OgreRenderSystem.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgrePrerequisites.h>
#include <OGRE/OgreAny.h>
#include <boost/algorithm/string/predicate.hpp>
#include <ros/ros.h>

RayCastUtils::RayCastUtils(Ogre::SceneManager* sm)
    : scene_manager_(sm)
{
    // Create the ray scene query object
    ray_scene_query_ = scene_manager_->createRayQuery(Ogre::Ray());//, Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK);
    if (ray_scene_query_ != NULL)
    {
        ray_scene_query_->setSortByDistance(true);
    }
}


RayCastUtils::~RayCastUtils()
{
    scene_manager_->destroyQuery(ray_scene_query_);
}

bool RayCastUtils::RayCastFromPoint(const Ogre::Vector3 &point, const Ogre::Vector3 &direction, Ogre::Vector3 frame_pos, Ogre::Quaternion frame_quat, Ogre::Vector3 &result, int &object_type, std::string &object_name)
{
    // create the ray to test
    Ogre::Ray ray(Ogre::Vector3(point.x, point.y, point.z),
                  Ogre::Vector3(direction.x, direction.y, direction.z));

    return RayCastFromPoint(ray,frame_pos,frame_quat,result,object_type,object_name);
}

// Raycast from a point in to the scene.
// returns success or failure.
// on success the point is returned in the result.
bool RayCastUtils::RayCastFromPoint(const Ogre::Ray ray, Ogre::Vector3 frame_pos, Ogre::Quaternion frame_quat, Ogre::Vector3 &result, int &object_type, std::string &object_name)
{
    // check we are initialised
    if (ray_scene_query_ != NULL)
    {
        ray_scene_query_->clearResults();

        // create a query object
        ray_scene_query_->setRay(ray);

        try
        {
            // execute the query, returns a vector of hits
            if (ray_scene_query_->execute().size() <= 0)
            {
                // raycast did not hit an objects bounding box
                return (false);
            }
        }
        catch (Ogre::Exception e)
        {
            e.getDescription();
        }
    }
    else
    {
        std::cout << "Cannot raycast without RaySceneQuery instance in RayCastUtils" << std::endl;
        return (false);
    }



    // at this point we have raycast to a series of different objects bounding boxes.
    // we need to test these different objects to see which is the first polygon hit.
    // there are some minor optimizations (distance based) that mean we wont have to
    // check all of the objects most of the time, but the worst case scenario is that
    // we need to test every triangle of every object.
    std::map<size_t,Ogre::Real> entity_closest_distance_map_;
    Ogre::Real closest_distance = -1.0f;
    Ogre::Vector3 closest_result;
    Ogre::RaySceneQueryResult &query_result = ray_scene_query_->getLastResults();
    for (size_t qr_idx = 0; qr_idx < query_result.size(); qr_idx++)
    {
        // stop checking if we have found a raycast hit that is closer
        // than all remaining entities
//        if ((closest_distance >= 0.0f) &&
//            (closest_distance < query_result[qr_idx].distance))
//        {
//            break;
//        }

        if(query_result[qr_idx].movable != NULL)
        {
            //query_result[qr_idx].movable->getParentSceneNode()->showBoundingBox(true);
            std::cout << query_result[qr_idx].movable->getName() << " of type " << query_result[qr_idx].movable->getMovableType() << std::endl;;
        }

        // only check this result if its a hit against an entity
        if ((query_result[qr_idx].movable != NULL) &&
            (query_result[qr_idx].movable->getMovableType().compare("Entity") == 0))
        {
            // get the entity to check
            Ogre::Entity *pentity = static_cast<Ogre::Entity*>(query_result[qr_idx].movable);

            // mesh data to retrieve
            size_t vertex_count;
            size_t index_count;
            Ogre::Vector3 *vertices;
            unsigned long *indices;

            // test for hitting individual triangles on the mesh

            // get the mesh information
            GetMeshInformationOptimized(pentity->getMesh(), vertex_count, vertices, index_count, indices,
                                        pentity->getParentNode()->_getDerivedPosition(), pentity->getParentNode()->_getDerivedOrientation(), pentity->getParentNode()->_getDerivedScale());

            Ogre::Real entity_closest_distance = -1.0f;
            for (int i = 0; i < static_cast<int>(index_count); i += 3)
            {
                // check for a hit against this triangle
                std::pair<bool, Ogre::Real> hit = Ogre::Math::intersects(ray, vertices[indices[i]],
                                                                         vertices[indices[i+1]], vertices[indices[i+2]], true, true);

                // if it was a hit check if its the closest
                if (hit.first)
                {
                    if ((entity_closest_distance < 0.0f) ||
                            (hit.second < entity_closest_distance))
                    {
                        // this is the closest so far, save it off
                        entity_closest_distance = hit.second;
                    }
                }
            }

            if(entity_closest_distance > 0)
                entity_closest_distance_map_[qr_idx] = entity_closest_distance; // save in map to check priority

            if ((closest_distance < 0.0f) ||
                (entity_closest_distance < closest_distance))
            {
                // this is the closest so far, save it off
                closest_distance = entity_closest_distance;
            }

            // free the verticies and indicies memory
            delete[] vertices;
            delete[] indices;
        }
        else if ((query_result[qr_idx].movable != NULL) &&
                 (query_result[qr_idx].movable->getMovableType().compare("PointCloudCustom") == 0))
        {
            // get the entity to check
            rviz::PointCloudCustom *pentity = static_cast<rviz::PointCloudCustom*>(query_result[qr_idx].movable);

            // mesh data to retrieve
            std::vector<Ogre::Vector3> vertices;
            std::vector<unsigned long> indices;

            // get the mesh information
            GetMeshInformationPointCloud(pentity, vertices, indices,
                                         frame_pos, frame_quat);
            std::cout << "all mesh information ready (" << vertices.size() << ", " << indices.size() << ")" << std::endl;

            // test for hitting individual triangles on the mesh
            Ogre::Real entity_closest_distance = -1.0f;
            for (int i = 0; i < indices.size(); i += 3)
            {
                //std::cout << "testing hit with " << i << " / " << vertices.size() << std::endl;
                //std::cout << "\t" << vertices[indices[i]].x << ", " << vertices[indices[i]].y << ", " << vertices[indices[i]].z << std::endl;
                //std::cout << "\t" << vertices[indices[i+1]].x << ", " << vertices[indices[i+1]].y << ", " << vertices[indices[i+1]].z << std::endl;
                //std::cout << "\t" << vertices[indices[i+2]].x << ", " << vertices[indices[i+2]].y << ", " << vertices[indices[i+2]].z << std::endl;
                // check for a hit against this triangle
                std::pair<bool, Ogre::Real> hit = Ogre::Math::intersects(ray, vertices[indices[i]],
                                                                         vertices[indices[i+1]], vertices[indices[i+2]], true, true);

                // if it was a hit check if its the closest
                if (hit.first)
                {
                    std::cout << "new hit: " << hit.second << "\n   compared to current closest hit: " << entity_closest_distance << std::endl;
                    if ((entity_closest_distance < 0.0f) ||
                        (hit.second < entity_closest_distance))
                    {
                        // this is the closest so far, save it off
                        entity_closest_distance = hit.second;
                    }
                }
            }
            std::cout << "tested for hits" << std::endl;

            if(entity_closest_distance > 0)
                entity_closest_distance_map_[qr_idx] = entity_closest_distance; // save in map to check priority

            if ((closest_distance < 0.0f) ||
                (entity_closest_distance < closest_distance))
            {
                // this is the closest so far, save it off
                closest_distance = entity_closest_distance;
            }

            // free the verticies and indicies memory
            vertices.clear();
            indices.clear();
			std::cout << "cleared memory" << std::endl;
        }
    }

    // now that we have all the per-triangle intersection, we do selection priority checking
    if(entity_closest_distance_map_.size() > 0)
    {
        // use the first one, which should be the closest point
        std::map<size_t,Ogre::Real>::iterator closest_object = entity_closest_distance_map_.begin();

        size_t index = closest_object->first;
        Ogre::MovableObject* movable = query_result[index].movable;
        Ogre::Real distance = closest_object->second;
        // get user-defined data (in this case, usually identifier
        std::string user_data = "";
        try
        {
            Ogre::Any any = movable->getUserAny();
            user_data = Ogre::any_cast<std::string>(any);
        }
        catch(...)
        {
            user_data = movable->getName();
        }

        // check if there is a need to iterate
        // ignore ground and footstep body bounding boxes
        while((boost::algorithm::starts_with(user_data,"ground plane") || boost::algorithm::starts_with(user_data,"body_bb")) && closest_object != entity_closest_distance_map_.end())
        {
            //ROS_INFO("ignore ground and footstep body bounding boxes (%s)",user_data.c_str());

            ++closest_object;

            index = closest_object->first;
            movable = query_result[index].movable;
            distance = closest_object->second;
            // get user-defined data (in this case, usually identifier
            user_data = "";
            try
            {
               Ogre::Any any = movable->getUserAny();
               user_data = Ogre::any_cast<std::string>(any);
            }
            catch(...)
            {
               user_data = movable->getName();
            }
        }

        // if we're intersecting a bounding object, need to prioritize template if template is there
        if(boost::algorithm::starts_with(movable->getName(),"BoundingObject"))
        {
            //ROS_INFO("we're intersecting a bounding object, prioritize template if template is there");
            for (std::map<size_t,Ogre::Real>::iterator it = closest_object; it != entity_closest_distance_map_.end(); ++it)
            {
                size_t new_index = it->first;
                Ogre::MovableObject* new_movable = query_result[new_index].movable;
                Ogre::Real new_distance = it->second;
                std::string new_user_data = "";
                try
                {
                   Ogre::Any any = new_movable->getUserAny();
                   new_user_data = Ogre::any_cast<std::string>(any);
                }
                catch(...)
                {
                   new_user_data = new_movable->getName();
                }

                //ROS_INFO("    [%d].%f: %s::%s",new_index,new_distance,new_movable->getName().c_str(),new_user_data.c_str());

                if(boost::algorithm::starts_with(user_data,"ground plane") || boost::algorithm::starts_with(new_user_data,"body_bb"))
                    continue;

                // if outside the bounding sphere or collided with robot mesh
                if(new_distance-distance > 0.2 || new_movable->getName().find("Robot") != std::string::npos)
                    break;

                if(boost::algorithm::starts_with(new_movable->getName(),"template"))
                {
                    //ROS_INFO("    found");

                    index = new_index;
                    movable = new_movable;
                    distance = new_distance;
                    user_data = new_user_data;
                    closest_object = it;
                    break;
                }
            }
        }
        // prioritize footsteps over step plan markers
        if(boost::algorithm::starts_with(user_data,"/step_plan_"))
        {
            //ROS_INFO("prioritize footsteps over step plan markers");
            for (std::map<size_t,Ogre::Real>::iterator it = closest_object; it != entity_closest_distance_map_.end(); ++it)
            {
                size_t new_index = it->first;
                Ogre::MovableObject* new_movable = query_result[new_index].movable;
                Ogre::Real new_distance = it->second;
                std::string new_user_data = "";
                try
                {
                   Ogre::Any any = new_movable->getUserAny();
                   new_user_data = Ogre::any_cast<std::string>(any);
                }
                catch(...)
                {
                   new_user_data = new_movable->getName();
                }

                //ROS_INFO("    [%d].%f: %s::%s",new_index,new_distance,new_movable->getName().c_str(),new_user_data.c_str());

                if(boost::algorithm::starts_with(user_data,"ground plane") || boost::algorithm::starts_with(new_user_data,"body_bb"))
                    continue;

                // if there is a footstep
                if(boost::algorithm::starts_with(new_user_data,"footstep"))
                {
                    //ROS_INFO("    found");

                    index = new_index;
                    movable = new_movable;
                    distance = new_distance;
                    user_data = new_user_data;
                    closest_object = it;
                    break;
                }
            }
        }
        // prioritize footstep goal steps over regular footsteps
        if(boost::algorithm::starts_with(user_data,"footstep") && !boost::algorithm::starts_with(user_data,"footstep goal"))
        {
            //ROS_INFO("prioritize footstep goal steps over regular footsteps");
            for (std::map<size_t,Ogre::Real>::iterator it = closest_object; it != entity_closest_distance_map_.end(); ++it)
            {
                size_t new_index = it->first;
                Ogre::MovableObject* new_movable = query_result[new_index].movable;
                Ogre::Real new_distance = it->second;
                std::string new_user_data = "";
                try
                {
                   Ogre::Any any = new_movable->getUserAny();
                   new_user_data = Ogre::any_cast<std::string>(any);
                }
                catch(...)
                {
                   new_user_data = new_movable->getName();
                }

                //ROS_INFO("    [%d].%f: %s::%s",new_index,new_distance,new_movable->getName().c_str(),new_user_data.c_str());

                if(boost::algorithm::starts_with(user_data,"ground plane") || boost::algorithm::starts_with(new_user_data,"body_bb"))
                    continue;

                // if there is a footstep
                if(boost::algorithm::starts_with(new_user_data,"footstep goal"))
                {
                    //ROS_INFO("    found");

                    index = new_index;
                    movable = new_movable;
                    distance = new_distance;
                    user_data = new_user_data;
                    closest_object = it;
                    break;
                }
            }
        }

        //ROS_INFO("raycast[%d].%f: %s::%s",index,distance,movable->getName().c_str(),user_data.c_str());

        // if we found a new closest raycast for this object, update the
        // closest_result before moving on to the next object.
        closest_distance = distance;
        closest_result = ray.getPoint(closest_distance);

        // type 0 -> UNKNOWN ENTITY
        // type 1 -> POINT CLOUD
        // type 2 -> WAYPOINT
        // type 3 -> TEMPLATE
        // type 4 -> FOOTSTEP
        object_type = 0;
        if(movable->getMovableType().compare("PointCloudCustom") == 0)
        {
            object_type = 1;
            object_name = movable->getName();
        }
        else if(movable->getMovableType().compare("Entity") == 0)
        {
            if(boost::algorithm::starts_with(user_data,"waypoint"))
                object_type = 2;
            else if(boost::algorithm::starts_with(user_data,"template"))
                object_type = 3;
            else if(boost::algorithm::starts_with(user_data,"footstep goal"))
                object_type = 4;
            else if(boost::algorithm::starts_with(user_data,"footstep"))
                object_type = 4;
            object_name = user_data;
        }
    }

    // return the result
    if (closest_distance >= 0.0f)
    {
        // raycast success
        result = closest_result;
        return (true);
    }
    else
    {
        // raycast failed
        return (false);
    }
}

void RayCastUtils::GetMeshInformationPointCloud(rviz::PointCloudCustom* &mesh, std::vector<Ogre::Vector3> &vertices, std::vector<unsigned long> &indices,
                                                Ogre::Vector3 frame_pos, Ogre::Quaternion frame_quat)
{
    mesh->getMesh(vertices,indices);
    
    for( int j = 0; j < vertices.size(); ++j)
    {
        Ogre::Vector3 pt = vertices[j];
        //pt = (orient * (pt * scale)) + position;
        vertices[j] = (frame_quat * pt) + frame_pos;
    }
}



//////////////////////////////////////////////////////////////////////////
// This code is a copy of RetrieveVertexData.(Optimized version)
// Original version
// Get the mesh information for the given mesh.
// Code found in Wiki: www.ogre3d.org/wiki/index.php/RetrieveVertexData

void RayCastUtils::GetMeshInformationOptimized(const Ogre::MeshPtr mesh, size_t &vertex_count, Ogre::Vector3* &vertices, size_t &index_count,
                                               unsigned long* &indices, const Ogre::Vector3 &position,	const Ogre::Quaternion &orient,	const Ogre::Vector3 &scale)
{
    bool added_shared = false;
    size_t current_offset = 0;
    size_t shared_offset = 0;
    size_t next_offset = 0;
    size_t index_offset = 0;

    vertex_count = index_count = 0;

    // Calculate how many vertices and indices we're going to need
    for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh( i );

        // We only need to add the shared vertices once
        if(submesh->useSharedVertices)
        {
            if( !added_shared )
            {
                vertex_count += mesh->sharedVertexData->vertexCount;
                added_shared = true;
            }
        }
        else
        {
            vertex_count += submesh->vertexData->vertexCount;
        }

        // Add the indices
        index_count += submesh->indexData->indexCount;
    }


    // Allocate space for the vertices and indices
    vertices = new Ogre::Vector3[vertex_count];
    indices = new unsigned long[index_count];

    added_shared = false;

    // Run through the submeshes again, adding the data into the arrays
    for ( unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);

        Ogre::VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;

        if((!submesh->useSharedVertices)||(submesh->useSharedVertices && !added_shared))
        {
            if(submesh->useSharedVertices)
            {
                added_shared = true;
                shared_offset = current_offset;
            }

            const Ogre::VertexElement* posElem =
                    vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);

            Ogre::HardwareVertexBufferSharedPtr vbuf =
                    vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

            unsigned char* vertex =
                    static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

            // There is _no_ baseVertexPointerToElement() which takes an Ogre::Real or a double
            //  as second argument. So make it float, to avoid trouble when Ogre::Real will
            //  be comiled/typedefed as double:
            //      Ogre::Real* pReal;
            float* pReal;

            for( size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
            {
                posElem->baseVertexPointerToElement(vertex, &pReal);

                Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);

                vertices[current_offset + j] = (orient * (pt * scale)) + position;
            }

            vbuf->unlock();
            next_offset += vertex_data->vertexCount;
        }


        Ogre::IndexData* index_data = submesh->indexData;
        size_t numTris = index_data->indexCount / 3;
        Ogre::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;

        bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);

        unsigned long*  pLong = static_cast<unsigned long*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
        unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);


        size_t offset = (submesh->useSharedVertices)? shared_offset : current_offset;
        size_t index_start = index_data->indexStart;
        size_t last_index = numTris*3 + index_start;

        if (use32bitindexes)
            for (size_t k = index_start; k < last_index; ++k)
            {
                indices[index_offset++] = pLong[k] + static_cast<unsigned long>( offset );
            }

        else
            for (size_t k = index_start; k < last_index; ++k)
            {
                indices[ index_offset++ ] = static_cast<unsigned long>( pShort[k] ) +
                        static_cast<unsigned long>( offset );
            }

        ibuf->unlock();
        current_offset = next_offset;
    }
}

//////////////////////////////////////////////////////////////////////////
// Adapted version
// The following is an adapted version of GetMeshInformation, provided by Rumi, that takes into account the entity in its currently animated state.

void RayCastUtils::GetMeshInformationForAnimations(const Ogre::Entity *entity, size_t &vertex_count, Ogre::Vector3* &vertices, size_t &index_count,
                                                   unsigned long* &indices, const Ogre::Vector3 &position,	const Ogre::Quaternion &orient,	const Ogre::Vector3 &scale)
{
    bool added_shared = false;
    size_t current_offset = 0;
    size_t shared_offset = 0;
    size_t next_offset = 0;
    size_t index_offset = 0;
    vertex_count = index_count = 0;

    Ogre::MeshPtr mesh = entity->getMesh();


    bool useSoftwareBlendingVertices = entity->hasSkeleton();

    if (useSoftwareBlendingVertices)
    {
        //Entity aEnt = &entity;
        //aEnt._updateAnimation();
    }

    // Calculate how many vertices and indices we're going to need
    for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh( i );

        // We only need to add the shared vertices once
        if(submesh->useSharedVertices)
        {
            if( !added_shared )
            {
                vertex_count += mesh->sharedVertexData->vertexCount;
                added_shared = true;
            }
        }
        else
        {
            vertex_count += submesh->vertexData->vertexCount;
        }

        // Add the indices
        index_count += submesh->indexData->indexCount;
    }


    // Allocate space for the vertices and indices
    vertices = new Ogre::Vector3[vertex_count];
    indices = new unsigned long[index_count];

    added_shared = false;

    // Run through the submeshes again, adding the data into the arrays
    for ( unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);

        //----------------------------------------------------------------
        // GET VERTEXDATA
        //----------------------------------------------------------------

        //Ogre::VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;
        Ogre::VertexData* vertex_data;

        //When there is animation:
        if(useSoftwareBlendingVertices)
#ifdef BUILD_AGAINST_AZATHOTH
            vertex_data = submesh->useSharedVertices ? entity->_getSharedBlendedVertexData() : entity->getSubEntity(i)->_getBlendedVertexData();
#else
            vertex_data = submesh->useSharedVertices ? entity->_getSkelAnimVertexData() : entity->getSubEntity(i)->_getSkelAnimVertexData();
#endif
        else
            vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;


        if((!submesh->useSharedVertices)||(submesh->useSharedVertices && !added_shared))
        {
            if(submesh->useSharedVertices)
            {
                added_shared = true;
                shared_offset = current_offset;
            }

            const Ogre::VertexElement* posElem =
                    vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);

            Ogre::HardwareVertexBufferSharedPtr vbuf =
                    vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

            unsigned char* vertex =
                    static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

            // There is _no_ baseVertexPointerToElement() which takes an Ogre::Real or a double
            //  as second argument. So make it float, to avoid trouble when Ogre::Real will
            //  be comiled/typedefed as double:
            //      Ogre::Real* pReal;
            float* pReal;

            for( size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
            {
                posElem->baseVertexPointerToElement(vertex, &pReal);

                Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);

                vertices[current_offset + j] = (orient * (pt * scale)) + position;
            }

            vbuf->unlock();
            next_offset += vertex_data->vertexCount;
        }


        Ogre::IndexData* index_data = submesh->indexData;
        size_t numTris = index_data->indexCount / 3;
        Ogre::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;

        bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);

        unsigned long*  pLong = static_cast<unsigned long*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
        unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);


        size_t offset = (submesh->useSharedVertices)? shared_offset : current_offset;
        size_t index_start = index_data->indexStart;
        size_t last_index = numTris*3 + index_start;

        if (use32bitindexes)
            for (size_t k = index_start; k < last_index; ++k)
            {
                indices[index_offset++] = pLong[k] + static_cast<unsigned long>( offset );
            }

        else
            for (size_t k = index_start; k < last_index; ++k)
            {
                indices[ index_offset++ ] = static_cast<unsigned long>( pShort[k] ) +
                        static_cast<unsigned long>( offset );
            }

        ibuf->unlock();
        current_offset = next_offset;
    }
}

//////////////////////////////////////////////////////////////////////////
// The versions of GetMeshInformation() fails on 64 bit machines using GCC. The following version, adapted from Rumi's version, solves this issue for me:

void RayCastUtils::GetMeshInformation64bit(Ogre::Entity *entity, size_t &vertex_count, Ogre::Vector3* &vertices, size_t &index_count, Ogre::uint32* &indices,
                                           const Ogre::Vector3 &position, const Ogre::Quaternion &orient, const Ogre::Vector3 &scale)
{
    bool added_shared = false;
    size_t current_offset = 0;
    size_t shared_offset = 0;
    size_t next_offset = 0;
    size_t index_offset = 0;
    vertex_count = index_count = 0;

    Ogre::MeshPtr mesh = entity->getMesh();


    bool useSoftwareBlendingVertices = entity->hasSkeleton();

    if (useSoftwareBlendingVertices)
    {
        entity->_updateAnimation();
    }

    // Calculate how many vertices and indices we're going to need
    for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh( i );

        // We only need to add the shared vertices once
        if(submesh->useSharedVertices)
        {
            if( !added_shared )
            {
                vertex_count += mesh->sharedVertexData->vertexCount;
                added_shared = true;
            }
        }
        else
        {
            vertex_count += submesh->vertexData->vertexCount;
        }

        // Add the indices
        index_count += submesh->indexData->indexCount;
    }


    // Allocate space for the vertices and indices
    vertices = new Ogre::Vector3[vertex_count];
    indices = new Ogre::uint32[index_count];

    added_shared = false;

    // Run through the submeshes again, adding the data into the arrays
    for ( unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);

        //----------------------------------------------------------------
        // GET VERTEXDATA
        //----------------------------------------------------------------
        Ogre::VertexData* vertex_data;

        //When there is animation:
        if(useSoftwareBlendingVertices)
            vertex_data = submesh->useSharedVertices ? entity->_getSkelAnimVertexData() : entity->getSubEntity(i)->_getSkelAnimVertexData();
        else
            vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;


        if((!submesh->useSharedVertices)||(submesh->useSharedVertices && !added_shared))
        {
            if(submesh->useSharedVertices)
            {
                added_shared = true;
                shared_offset = current_offset;
            }

            const Ogre::VertexElement* posElem =
                    vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);

            Ogre::HardwareVertexBufferSharedPtr vbuf =
                    vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

            unsigned char* vertex =
                    static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

            // There is _no_ baseVertexPointerToElement() which takes an Ogre::Real or a double
            //  as second argument. So make it float, to avoid trouble when Ogre::Real will
            //  be comiled/typedefed as double:
            //      Ogre::Real* pReal;
            float* pReal;

            for( size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
            {
                posElem->baseVertexPointerToElement(vertex, &pReal);

                Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);

                vertices[current_offset + j] = (orient * (pt * scale)) + position;
            }

            vbuf->unlock();
            next_offset += vertex_data->vertexCount;
        }


        Ogre::IndexData* index_data = submesh->indexData;
        size_t numTris = index_data->indexCount / 3;
        Ogre::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;

        bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);

        void* hwBuf = ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY);

        size_t offset = (submesh->useSharedVertices)? shared_offset : current_offset;
        size_t index_start = index_data->indexStart;
        size_t last_index = numTris*3 + index_start;

        if (use32bitindexes) {
            Ogre::uint32* hwBuf32 = static_cast<Ogre::uint32*>(hwBuf);
            for (size_t k = index_start; k < last_index; ++k)
            {
                indices[index_offset++] = hwBuf32[k] + static_cast<Ogre::uint32>( offset );
            }
        } else {
            Ogre::uint16* hwBuf16 = static_cast<Ogre::uint16*>(hwBuf);
            for (size_t k = index_start; k < last_index; ++k)
            {
                indices[ index_offset++ ] = static_cast<Ogre::uint32>( hwBuf16[k] ) +
                        static_cast<Ogre::uint32>( offset );
            }
        }

        ibuf->unlock();
        current_offset = next_offset;
    }
}

