#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D node_interval = (end - start) / (num_nodes - 1);
        for(int i=0; i<num_nodes; i++)
        {
            Mass *node = new Mass(start + node_interval * i, node_mass, false);
            masses.push_back(node);
        }
        for(int i=0; i<num_nodes-1; i++)
        {
            Spring *node_interval_spring = new Spring(masses[i], masses[i+1], k);
            springs.push_back(node_interval_spring);
        }
//        Comment-in this part when you implement the constructor
       for (auto &i : pinned_nodes) {
           masses[i]->pinned = true;
       }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D ab_dir = s->m1->position - s->m2->position;
            Vector2D force_m2Tom1 = (-s->k) * (ab_dir / ab_dir.norm()) * (ab_dir.norm() - s->rest_length);
            s->m1->forces += force_m2Tom1;
            s->m2->forces += -force_m2Tom1;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                
                float kd = 0.01f;                                               //different kd have diffreent effect
                m->forces += gravity * m->mass - kd * m->velocity;
                // m->position = m->position + m->velocity * delta_t;                  //explict method
                m->velocity = m->velocity + (m->forces / m->mass) * delta_t;
                m->position = m->position + m->velocity * delta_t;              //semi implict method

                // TODO (Part 2): Add global damping
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D ab_dir = s->m1->position - s->m2->position;
            Vector2D force_m2Tom1 = (-s->k) * (ab_dir / ab_dir.norm()) * (ab_dir.norm() - s->rest_length);
            s->m1->forces += (force_m2Tom1);
            s->m2->forces += (-force_m2Tom1);
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // // TODO (Part 3.1): Set the new position of the rope mass
                float damping_factor = 0.00005f;
                m->forces += gravity * m->mass;
                Vector2D a = m->forces / m->mass;
                m->position += (1 - damping_factor) * (m->position - m->last_position) + a * delta_t * delta_t;
                m->last_position = temp_position;
                // TODO (Part 4): Add global Verlet damping
            }

            m->forces = Vector2D(0,0);
        }
    }
}
