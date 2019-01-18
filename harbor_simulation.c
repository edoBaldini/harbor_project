#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>
#include <allegro5/allegro5.h>
#include <allegro5/allegro_image.h>
#include <allegro5/allegro_primitives.h>
#include <pthread.h>
#include "ptask.h"

//------------------------------------------------------------------------------
// GLOBAL CONSTANTS
//------------------------------------------------------------------------------
#define XWIN            2560        // width monitor
#define YWIN            1980        // height monitor
#define PERIOD          20          // in ms
#define DLINE           60
#define PRIO            10
#define XPORT           1280.f
#define YPORT           1500.f
#define VEL             60
#define FPS             60.0
#define FRAME_PERIOD    (1 / FPS)
#define EPSILON         2.f        // guardian distance to the goal
#define XSTARTPOS       300.f
#define YSTARTPOS       1980.f
//------------------------------------------------------------------------------
// GLOBAL STRUCTURE
//------------------------------------------------------------------------------
typedef struct SHIP
{
    float x, y;
    float width, height;
    float vel;
    float bow_grade;
    float traj_grade; 
} SHIP;

//------------------------------------------------------------------------------
// GLOBAL VARIABLES
//------------------------------------------------------------------------------
bool    DONE    = false;
bool    REDRAW  = true;
SHIP    titanic;
ALLEGRO_BITMAP * port;

float distance_vector(float x_start, float y_start, float x_target, float y_target)
{
    float x_m = x_target - x_start;
    float y_m = y_target - y_target;
    
    return sqrtf((x_m * x_m) + (y_m * y_m));
}

float degree_rect(float x1, float y1, float x2, float y2)
{
    float angular_coefficient   = (x1 == x2) ? x1 : ((y2 - y1) / (x2 - x1));
    float degree                = atan(angular_coefficient);
    return (x2 <= x1) ? degree + ALLEGRO_PI  : degree + 2 * ALLEGRO_PI;
}

void must_init(bool test, const char *description)
{
    if(test) return;

    printf("couldn't initialize %s\n", description);
    exit(1);
}

bool linear_movement(float xtarget_pos,float ytarget_pos, bool reg_vel)
{   
    
    float velx;
    float vely;
    float vel;
    titanic.bow_grade        = degree_rect(titanic.x, titanic.y, xtarget_pos, ytarget_pos);
    if (reg_vel)
    {
        velx = 2 * (xtarget_pos - titanic.x);
        vely = 2 * (ytarget_pos - titanic.y);
        vel = (sqrtf(velx * velx + vely * vely));
        titanic.vel = (fabs(vel) > VEL) ? VEL : vel;
    }
        
    if (fabs(xtarget_pos - titanic.x) <= EPSILON)
        return true;
    else 
        titanic.x = titanic.x + (titanic.vel * cos(titanic.bow_grade) / PERIOD);

    if (fabs(ytarget_pos - titanic.y) <= EPSILON )
    {   
        return true;
    }
    else 
        titanic.y = titanic.y + (titanic.vel * sin(titanic.bow_grade) / PERIOD);

    if (titanic.x == xtarget_pos && titanic.y == ytarget_pos)
    {   
        return true;
    }
    else 
        return false;
}

void * task(void * arg)
{
    bool first_step_done = false;
    bool second_step_done = false;
    float route[] = {   
                        500, 1940,
                        688, 1819, 
                        879, 1719,
                        1217, 1685,
                        1257, 1600,
                        XPORT, YPORT

                    };
    bool reached[] = {
                        false, false,
                        false, false,
                        false, false,
                        false, false,
                        false, false,
                        false, false
                    };
    // Task private variables
    const int id = ptask_id(arg);
    ptask_activate(id);
    int i = 0;
    int len_route = 12;
    while (!DONE) {

        if (i < len_route)
        {   
           if (!reached[i])
            {  // titanic.bow_grade        = degree_rect(titanic.x, titanic.y, route[i], route[i+1]);
                bool need_stop = (i + 1) == (len_route - 1); 
                reached[i] = linear_movement(route[i], route[i+1], need_stop);
            }
            else
                i = i + 2;
        }

        if (ptask_deadline_miss(id))
        {   
            printf("%d) deadline missed!\n", id);
        }
        ptask_wait_for_activation(id);

    }

    return NULL;
}



int main()
{
int xport = 0; // position of the port on x axis
int yport = 0; // position of the port on y axis
    must_init(al_init(), "allegro");
    must_init(al_install_keyboard(), "keyboard");

    ALLEGRO_PATH *path = al_get_standard_path(ALLEGRO_RESOURCES_PATH);
    al_append_path_component(path, "img");
    al_change_directory(al_path_cstr(path, '/'));  // change the working directory
    al_destroy_path(path);

    ALLEGRO_TIMER* timer = al_create_timer(FRAME_PERIOD);
    must_init(timer, "timer");

    ALLEGRO_EVENT_QUEUE* queue = al_create_event_queue();
    must_init(queue, "queue");

    al_set_new_display_option(ALLEGRO_SAMPLE_BUFFERS, 1, ALLEGRO_SUGGEST);
    al_set_new_display_option(ALLEGRO_SAMPLES, 8, ALLEGRO_SUGGEST);
    al_set_new_bitmap_flags(ALLEGRO_MIN_LINEAR | ALLEGRO_MAG_LINEAR);

    ALLEGRO_DISPLAY* disp = al_create_display(XWIN, YWIN);
    must_init(disp, "display");

    must_init(al_init_image_addon(), "image addon");
    
    port = al_load_bitmap("port.png");
    must_init(port, "port");
    xport = XWIN - al_get_bitmap_width(port);

    ALLEGRO_BITMAP* ship = al_load_bitmap("ship.png");
    must_init(ship, "ship");
    must_init(al_init_primitives_addon(), "primitives");

    al_register_event_source(queue, al_get_keyboard_event_source());
    al_register_event_source(queue, al_get_display_event_source(disp));
    al_register_event_source(queue, al_get_timer_event_source(timer));

    ALLEGRO_EVENT event;

    titanic.x = XSTARTPOS; //rand() % 640;
    titanic.y = YSTARTPOS;//rand() % 480;
    titanic.vel = VEL;
    titanic.width = al_get_bitmap_width(ship);
    titanic.height = al_get_bitmap_height(ship);

    ptask_create(task, PERIOD, DLINE, PRIO);
    al_start_timer(timer);
    while(1)
    {
        al_wait_for_event(queue, &event);

        switch(event.type)
        {
            case ALLEGRO_EVENT_TIMER:
                REDRAW = true;
                break;

            case ALLEGRO_EVENT_KEY_DOWN:
            case ALLEGRO_EVENT_DISPLAY_CLOSE:
                DONE = true;
                break;
        }

        if(DONE)
            break;

        if(REDRAW && al_is_event_queue_empty(queue))
        {
            al_clear_to_color(al_map_rgb(12, 87, 151));

            al_draw_bitmap(port, xport, yport, 0);
            al_draw_filled_circle(XPORT, YPORT, 3,al_map_rgb_f(1, 1, 1));
            al_draw_rotated_bitmap(ship, titanic.width / 2, 0, titanic.x, titanic.y, titanic.bow_grade + ALLEGRO_PI / 2, 0);

            al_flip_display();
            REDRAW = false;
        }
    }

    ptask_wait_tasks();

    al_destroy_bitmap(ship);
    al_destroy_bitmap(port);
    al_destroy_display(disp);
    al_destroy_timer(timer);
    al_destroy_event_queue(queue);

    return 0;
}

// void translation(float xtarget_pos, float ytarget_pos, )
// {
    // if (titanic.vel != VEL)   
    // {
        // float initial_degree        = degree_rect(titanic.x, titanic.y, xtarget_pos, ytarget_pos);
        // float desired_degree        = ALLEGRO_PI / 2;
        // float cur_offset            = (desired_degree + initial_degree);
        // float delta_alpha           = (xtarget_pos > titanic.x) ? cur_offset + titanic.bow_grade : cur_offset - titanic.bow_grade;
        // float estimated_cycle       = 50 * PERIOD;
        // delta_alpha = delta_alpha / estimated_cycle;
        // float guard_degree = (xtarget_pos < titanic.x) ? fmod(titanic.bow_grade, ALLEGRO_PI) : titanic.bow_grade - ALLEGRO_PI;
// 
        // if (guard_degree < desired_degree)
            // titanic.bow_grade -= delta_alpha;
// 
         // else if (guard_degree > desired_degree)
                 // titanic.bow_grade -= delta_alpha;
// 
//            else titanic.bow_grade = desired_degree;
// 
         // if (fabs(guard_degree - desired_degree) <= 0.2)
            // titanic.bow_grade = - desired_degree;
        // printf("guard %f, bow grade %f desired %f, delta alpha %f, difference degree %f\n", guard_degree, titanic.bow_grade, desired_degree, delta_alpha, fabs(guard_degree - desired_degree));
    // }
// 
// }
