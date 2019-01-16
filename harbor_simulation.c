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
#define YPORT           1445.f
#define VEL             300
#define FPS             60.0
#define FRAME_PERIOD    (1 / FPS)
#define EPSILON         1          // guardian distance to the goal
#define XSTARTPOS       20
#define YSTARTPOS       1980
//------------------------------------------------------------------------------
// GLOBAL STRUCTURE
//------------------------------------------------------------------------------
typedef struct SHIP
{
    float x, y;
    float width, height;
    float vel;
    float alpha; 
} SHIP;

//------------------------------------------------------------------------------
// GLOBAL VARIABLES
//------------------------------------------------------------------------------
bool    DONE    = false;
bool    REDRAW  = true;
SHIP    titanic;
ALLEGRO_BITMAP * port;


float distance_vector(x_start, y_start, x_target, y_target)
{
    float x_m = x_target - x_start;
    float y_m = y_target - y_target;

    return sqrtf((x_m * x_m) + (y_m * y_m));
}

void must_init(bool test, const char *description)
{
    if(test) return;

    printf("couldn't initialize %s\n", description);
    exit(1);
}


void * task(void * arg)
{
    // Task private variables
    bool is_reached = false;

    float angular_coefficient   = ((YPORT - titanic.y) / (XPORT - titanic.x));
    float initial_degree        = atan(angular_coefficient);
    float desired_degree        = (ALLEGRO_PI / 2);
    float delta_alpha           = desired_degree + initial_degree;
    float trajectory_alpha      = initial_degree;
    float x_average_velocity     = (2 * (XPORT - titanic.x));
    float y_average_velocity     = (2 * (YPORT - titanic.y));
    float average_velocity      = (sqrtf(x_average_velocity * x_average_velocity + y_average_velocity * y_average_velocity));
    float esitmated_trav_time;
    float estimated_cycle;
    float velx;
    float vely;
    float vel;
    float relative_vel;
    float degree_control;
    const int id = ptask_id(arg);
    
    titanic.alpha = (XPORT > titanic.x) ? initial_degree : initial_degree + ALLEGRO_PI;

    if (fabs(average_velocity - VEL) > 0)
        average_velocity = VEL;

    esitmated_trav_time = distance_vector(titanic.x, titanic.y, XPORT, YPORT) / average_velocity;

    estimated_cycle = esitmated_trav_time / FRAME_PERIOD;
    delta_alpha = (XPORT > titanic.x) ? delta_alpha / estimated_cycle : (-1) * delta_alpha / estimated_cycle;
    ptask_activate(id);

    
    while (!DONE) {

        if (!is_reached)
        {   
            velx = 2 * (XPORT - titanic.x);
            vely = 2 * (YPORT - titanic.y);
            vel = (sqrtf(velx * velx + vely * vely));
            relative_vel = (XPORT > titanic.x) ? vel : (-1) * vel;
            
            if (relative_vel < VEL)                                 
                titanic.vel = relative_vel;

            degree_control = (XPORT > titanic.x) ? titanic.alpha : -1*fmod(titanic.alpha, ALLEGRO_PI);
            if (degree_control >= -(desired_degree)) 
                titanic.alpha -= delta_alpha;

            titanic.x = titanic.x + (titanic.vel * cos(trajectory_alpha) * FRAME_PERIOD);
    
            if (fabs(XPORT - titanic.x) < EPSILON)
                titanic.x = XPORT;

            titanic.y = titanic.y + (titanic.vel * sin(trajectory_alpha) * FRAME_PERIOD);
    
            if (fabs(YPORT - titanic.y) < EPSILON)
                titanic.y = YPORT;

            if (titanic.x == XPORT && titanic.y == YPORT)
            {   
                is_reached = true;
            }
            if (ptask_deadline_miss(id))
            {   
                printf("%d) deadline missed!\n", id);
            }
            ptask_wait_for_activation(id);
        }
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
            al_draw_rotated_bitmap(ship, titanic.width / 2, 0, titanic.x, titanic.y, titanic.alpha + ALLEGRO_PI / 2, 0);
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