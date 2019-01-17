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
#define DLINE           35
#define PRIO            10
#define XPORT           1280.f
#define YPORT           1500.f
#define VEL             60
#define FPS             60.0
#define FRAME_PERIOD    (1 / FPS)
#define EPSILON         1.f        // guardian distance to the goal
#define XSTARTPOS       0.f
#define YSTARTPOS       1900.f
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
    return atan(angular_coefficient);
}

void must_init(bool test, const char *description)
{
    if(test) return;

    printf("couldn't initialize %s\n", description);
    exit(1);
}

bool linear_movement(float xtarget_pos,float ytarget_pos)
{   
    float initial_degree        = degree_rect(titanic.x, titanic.y, xtarget_pos, ytarget_pos);
    float velx = (xtarget_pos - titanic.x);
    float vely = (ytarget_pos - titanic.y);
    float vel = (sqrtf(velx * velx + vely * vely));

    titanic.vel = (fabs(vel) > VEL) ? VEL : vel;

    if (titanic.bow_grade == 0)
        titanic.bow_grade = (xtarget_pos <= titanic.x) ? initial_degree + ALLEGRO_PI  : initial_degree + 2 * ALLEGRO_PI;
    
    if (fabs(xtarget_pos - titanic.x) <= EPSILON)
        titanic.x = xtarget_pos;
    else 
        titanic.x = titanic.x + (titanic.vel * cos(titanic.bow_grade) / PERIOD);

    if (fabs(ytarget_pos - titanic.y) <= EPSILON)
        titanic.y = ytarget_pos;
    else 
        titanic.y = titanic.y + (titanic.vel * sin(titanic.bow_grade) / PERIOD);

    if (titanic.x == xtarget_pos && titanic.y == ytarget_pos)
    {   
        titanic.bow_grade = 0;
        return true;
    }
}

// void translation(float xtarget_pos, float ytarget_pos)
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

bool sub_interval(float xtarget, float ytarget, bool reach){

    linear_movement(xtarget, ytarget);
    if (xtarget == titanic.x  && ytarget == titanic.y) 
        return true;
    else  
        return false;
}

void * task(void * arg)
{
    bool first_step_done = false;
    bool second_step_done = false;
    // Task private variables
    const int id = ptask_id(arg);
    ptask_activate(id);

    while (!DONE) {
        if (!first_step_done)
            first_step_done = linear_movement(XPORT-300, YPORT+300);

        else
            second_step_done = linear_movement(XPORT, YPORT);
        if (second_step_done)
            titanic.bow_grade = - ALLEGRO_PI /2;
        if (ptask_deadline_miss(id))
        {   
            printf("%d) deadline missed!\n", id);
        }
        ptask_wait_for_activation(id);
    }

    // Task private variables cleanup

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