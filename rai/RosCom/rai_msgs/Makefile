BASE   = ../../..
NAME   = $(shell basename `pwd`)

include $(BASE)/build/config.mk

ROSDIR = /opt/ros/$(ROS_DISTRO)

SRCS = $(shell find msg -maxdepth 1 -name '*.msg' )
#SRCS := $(SR) $(shell find srv -maxdepth 1 -name '*.srv' )
OBJS = $(SRCS:msg/%.msg=%.h) StringString.h

default: $(OBJS)

clean: ;
	rm -f *.h src/$(NAME)/msg/*.py

%.h: msg/%.msg ;
	python2 $(ROSDIR)/lib/gencpp/gen_cpp.py \
         $< \
         -o . \
         -p $(NAME) \
         -e $(ROSDIR)/share/gencpp \
         -I "$(NAME):$(PWD)/msg" \
         -I "std_msgs:$(ROSDIR)/share/std_msgs/msg" \
         -I "geometry_msgs:$(ROSDIR)/share/geometry_msgs/msg"
	python2 $(ROSDIR)/lib/genpy/genmsg_py.py \
         $< \
         -o src/$(NAME)/msg \
         -p $(NAME) \
         -I "$(NAME):$(PWD)/msg" \
         -I "std_msgs:$(ROSDIR)/share/std_msgs/msg" \
         -I "geometry_msgs:$(ROSDIR)/share/geometry_msgs/msg"

%.h: srv/%.srv ;
	python2 $(ROSDIR)/lib/gencpp/gen_cpp.py \
         $< \
         -o . \
         -p $(NAME) \
         -e $(ROSDIR)/share/gencpp \
         -I "$(NAME):$(PWD)/msg" \
         -I "std_msgs:$(ROSDIR)/share/std_msgs/msg" \
         -I "geometry_msgs:$(ROSDIR)/share/geometry_msgs/msg"

#include $(BASE)/build/generic.mk

