CC		= gcc
CFLAGS	= --std=c99 -g -Wall
INCLUDES= -I.
LIBS	= -lm -lallegro -lallegro_main -lallegro_image -lallegro_primitives -lallegro_font -lpthread
SRCS	= harbor_simulation.c ptask.c 
OBJS	= $(SRCS:.c=.o)
MAIN	= harbor_simulation

all: $(MAIN)

$(MAIN): $(OBJS)
	$(CC) -o $@ $^ $(INCLUDES) $(LIBS) $(CFLAGS)

.c.o:
	$(CC) -c $< -o $@ $(CFLAGS)


.PHONY: clean
clean:
	$(RM) *.o *~ $(MAIN) $(BQUEUE_TEST)
