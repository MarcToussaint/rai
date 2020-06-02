BASE   = ../../..
NAME   = $(shell basename `pwd`)

include $(BASE)/build/config.mk

ROSDIR = /opt/ros/$(ROS_DISTRO)

SRCS_msg = $(shell find msg -maxdepth 1 -name '*.msg' )
SRCS_srv = $(shell find srv -maxdepth 1 -name '*.srv' )
SRCS = $(SRCS_msg) $(SRCS_srv)
OBJS = $(SRCS_msg:msg/%.msg=%.h) $(SRCS_srv:srv/%.srv=%.h)

target: $(OBJS)

CLEAN = *.h src/$(NAME)/msg/_[A-Za-z]*.py src/$(NAME)/srv/_[A-Za-z]*.py

%.h: msg/%.msg ;
	python2 $(ROSDIR)/lib/gencpp/gen_cpp.py \
         $< \
         -o . \
         -p $(NAME) \
         -e $(ROSDIR)/share/gencpp \
         -I "$(NAME):$(PWD)/msg" \
         -I "std_msgs:$(ROSDIR)/share/std_msgs/msg" \
         -I "geometry_msgs:$(ROSDIR)/share/geometry_msgs/msg" \
         -I "trajectory_msgs:$(ROSDIR)/share/trajectory_msgs/msg"
	python2 $(ROSDIR)/lib/genpy/genmsg_py.py \
         $< \
         -o src/$(NAME)/msg \
         -p $(NAME) \
         -I "$(NAME):$(PWD)/msg" \
         -I "std_msgs:$(ROSDIR)/share/std_msgs/msg" \
         -I "geometry_msgs:$(ROSDIR)/share/geometry_msgs/msg" \
         -I "trajectory_msgs:$(ROSDIR)/share/trajectory_msgs/msg"

%.h: srv/%.srv ;
	python2 $(ROSDIR)/lib/gencpp/gen_cpp.py \
         $< \
         -o . \
         -p $(NAME) \
         -e $(ROSDIR)/share/gencpp \
         -I "$(NAME):$(PWD)/msg" \
         -I "std_msgs:$(ROSDIR)/share/std_msgs/msg" \
         -I "geometry_msgs:$(ROSDIR)/share/geometry_msgs/msg" \
         -I "trajectory_msgs:$(ROSDIR)/share/trajectory_msgs/msg"
	python2 $(ROSDIR)/lib/genpy/gensrv_py.py \
         $< \
         -o src/$(NAME)/srv \
         -p $(NAME) \
         -I "$(NAME):$(PWD)/msg" \
         -I "std_msgs:$(ROSDIR)/share/std_msgs/msg" \
         -I "geometry_msgs:$(ROSDIR)/share/geometry_msgs/msg" \
         -I "trajectory_msgs:$(ROSDIR)/share/trajectory_msgs/msg"

include $(BASE)/build/generic.mk

