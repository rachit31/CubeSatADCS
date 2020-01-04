NAME := KalmanPi
LIBS := -lpthread
FLAGS :=
SRC := KalmanPi.cpp i2c.cpp mutex.cpp sleepTimer.cpp sendUdp.cpp readGyros.cpp readAccel.cpp readMag.cpp sendFunction.cpp threadManager.cpp thread.cpp propagateANDupdate.cpp 
BUILD_DIR := ./build/
OBJ_DIR := $(BUILD_DIR)obj/

# All Target
all: main-build

# Make build and obj directories
pre-build:
	mkdir -p ./build/obj

main-build: pre-build $(NAME)

# Compile src files to objects
$(SRC:.cpp=.o): $(SRC)
	g++ $(FLAGS) $(LIBS) -c $(@:.o=.cpp) -o $(OBJ_DIR)$(notdir $@)

# Tool invocations
$(NAME): $(SRC:.cpp=.o)
	g++ $(FLAGS) $(LIBS) -I$(OBJ_DIR) $(addprefix $(OBJ_DIR),$(notdir $(SRC:.cpp=.o))) -o $(BUILD_DIR)$(NAME)

# Other Targets
clean:
	rm -rf ./build
