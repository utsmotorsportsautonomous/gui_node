import pygame
import cv2
import numpy as np
import imutils
import thorpy


# Links for Future Use####
# http://www.thorpy.org/tutorials/include.html
# https://www.pygame.org/docs/
# https://www.reddit.com/r/pygame/comments/89ygm7/pygame_awesome_libraries/


class interface(object):
    def __init__(self):
        self.forward = False
        self.backward = False
        self.left = False
        self.right = False
        self.enable_control = False

        self.joystick = None
        self.joystick_name = None
        self.joystick_present = False
        self.axis0 = None
        self.axis1 = None
        self.axis2 = None
        self.axis3 = None
        self.axis4 = None
        self.axis5 = None

        self.lane1_button = None
        self.lane2_button = None
        self.object_button = None
        self.finish_button = None
        self.stop_button = None
        self.quit_button = None
        self.disable_obstacle_button = None

        self.last_button_pressed = 0

        self.lane1_enable = True
        self.lane2_enable = False
        self.object_enable = False
        self.finish_enable = False

        self.all_stop = False
        self.obstacle_disable = False
        self.color_update_ready = False
        self.exit_run = False

        self.HSV_Lane1_Upper = None
        self.HSV_Lane2_Upper = None
        self.HSV_Object_Upper = None
        self.HSV_Lane1_Lower = None
        self.HSV_Lane2_Lower = None
        self.HSV_Object_Lower = None
        self.HSV_Finish_Upper = None
        self.HSV_Finish_Lower = None

        self.Upper_Canny_Value = None
        self.Lower_Canny_Value = None

        self.slide_limit = 255

        self.menu = None
        self.menu_content = None
        self.menu_slide_reaction = None
        self.menu_inserter_reaction = None

        self.pressed = None

        self.text = None
        self.text_on_screen = None
        self.text_font_object = None
        self.text_font = 'arial'
        self.text_color = (255, 255, 255)

        self.screen = None
        self.frame = None
        self.screen_width = 1280
        self.screen_height = 720
        self.menu_size = 280
        self.frame_width = self.screen_width - self.menu_size
        self.screen_caption = "wasd car control"

        pygame.init()
        pygame.joystick.init()

        self.joystick_count = pygame.joystick.get_count()

    def lane1_update(self):
        self.lane1_enable = True
        self.lane2_enable = False
        self.object_enable = False
        self.finish_enable = False

    def lane2_update(self):
        self.lane1_enable = False
        self.lane2_enable = True
        self.object_enable = False
        self.finish_enable = False

    def object_update(self):
        self.lane1_enable = False
        self.lane2_enable = False
        self.object_enable = True
        self.finish_enable = False

    def finish_update(self):
        self.lane1_enable = False
        self.lane2_enable = False
        self.object_enable = False
        self.finish_enable = True

    def stop_update(self):
        if self.all_stop:
            self.all_stop = False
        else:
            self.all_stop = True

    def obstacle_update(self):
        if self.obstacle_disable:
            self.obstacle_disable = False
        else:
            self.obstacle_disable = True

    def create_menu(self):
        self.lane1_button = thorpy.make_button("Lane 1 Color Choice",
                                               func=self.lane1_update)

        self.lane2_button = thorpy.make_button("Lane 2 Color Choice",
                                               func=self.lane2_update)

        self.object_button = thorpy.make_button("Object Color Choice",
                                                func=self.object_update)

        self.finish_button = thorpy.make_button("Finish Line Color Choice",
                                                func=self.finish_update)

        self.disable_obstacle_button = thorpy.make_button(
                                               ("Disable Obstacle Detection"),
                                               func=self.obstacle_update)

        self.stop_button = thorpy.make_button("STOP!!",
                                              func=self.stop_update)

        self.quit_button = thorpy.make_button("Quit",
                                              func=thorpy.functions.quit_func)

        self.menu_content = thorpy.Box.make(elements=[
            self.lane1_button,
            self.lane2_button,
            self.object_button,
            self.finish_button,
            self.disable_obstacle_button,
            self.stop_button,
            self.quit_button])

        self.menu = thorpy.Menu(self.menu_content)

        for element in self.menu.get_population():
            element.surface = self.screen

        self.lane1_update()
        self.menu_content.set_topleft((self.frame_width, 100))
        self.menu_content.blit()
        self.menu_content.update()

    def get_all_stop(self):
        return self.all_stop

    def get_obstacle_detect(self):
        return self.obstacle_disable

    def color_update_ready_call(self):
        return self.color_update_ready

    def display_text(self, text, xpos, ypos, font_size):
        self.screen.fill(pygame.Color("black"), (xpos, ypos, 500, 100))
        self.text = str(text)
        self.text_font_object = pygame.font.SysFont(name=self.text_font,
                                                    size=font_size)
        self.text_on_screen = self.text_font_object.render(self.text,
                                                           True,
                                                           self.text_color)
        self.screen.blit(self.text_on_screen, (xpos, ypos))

    def update_frame(self, cv_image):
        cv_image = imutils.resize(cv_image, width=min(self.frame_width,
                                                      cv_image.shape[1]))
        self.frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.frame = cv2.flip(self.frame, 1)
        self.frame = np.rot90(self.frame)
        self.frame = pygame.surfarray.make_surface(self.frame)
        self.screen.blit(self.frame, (0, 0))
        #self.menu_content.blit()
        #self.menu_content.update()
        pygame.display.update()

    def process_events(self):
        pygame.event.pump()
        for event in pygame.event.get():
            #self.menu.react(event)
            if event.type == pygame.QUIT:
                self.exit_run = True

    def exit_check(self):
        return self.exit_run

    def start_screen(self):
        self.screen = pygame.display.set_mode(
            [self.screen_width, self.screen_height])
        pygame.display.set_caption(self.screen_caption)

    def init_joystick(self):
        self.joystick_count = pygame.joystick.get_count()
        if self.joystick_count is not 0:
            self.joystick_present = True
            self.joystick = [pygame.joystick.Joystick(x) for x in
                             range(self.joystick_count)]
            for x in range(self.joystick_count):
                self.joystick[x].init()
                self.joystick_name[x] = self.joystick[x].get_name()
        else:
            print("No Joystick Present")
            self.joystick_present = False

    def get_key_input(self):
        self.pressed = pygame.key.get_pressed()

        if (self.pressed[pygame.K_w]):
            self.last_button_pressed = 2
        elif (self.pressed[pygame.K_s]):
            self.last_button_pressed = 3
        else:
            self.last_button_pressed = self.last_button_pressed

        if (self.pressed[pygame.K_a]):
            self.last_button_pressed = 4
        elif (self.pressed[pygame.K_d]):
            self.last_button_pressed = 5
        else:
            self.last_button_pressed = self.last_button_pressed

        if (self.pressed[pygame.K_p]):
            self.last_button_pressed = 1
        elif (self.pressed[pygame.K_c]):
            self.last_button_pressed = 1
        else:
            self.last_button_pressed = 0

        return self.last_button_pressed

    def get_joystick_count(self):
        self.joystick_count = pygame.joystick.get_count()
        return self.joystick_count

    def check_joystick(self):
        return self.joystick_present

    def get_joystick_input(self, joystick_num):
        if self.joystick_present:
            # Left / Right on left joystick
            self.axis0 = self.joystick[joystick_num].get_axis(0)
            # Up / Dpwn on left joystick
            self.axis1 = self.joystick[joystick_num].get_axis(1)
            # l2 -1 -> 1 #
            # R2 / L2 L2 is positive, R2 is negative.
            self.axis2 = self.joystick[joystick_num].get_axis(2)

            self.axis3 = self.joystick[joystick_num].get_axis(3)

            self.axis4 = self.joystick[joystick_num].get_axis(4)

            # R2  -1 -> 1
            # ####/ L2 L2 is positive, R2 is negative.
            self.axis5 = self.joystick[joystick_num].get_axis(5)

            return(self.axis0,
                   self.axis1,
                   self.axis2,
                   self.axis3,
                   self.axis4,
                   self.axis5)
