import pygame

NORMAL = (128, 128, 128)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
BLUE = (0, 0, 255)

KEY_START = pygame.K_s
KEY_CONTINUE = pygame.K_c
KEY_QUIT_RECORDING = pygame.K_q
KEY_PAUSE = pygame.K_p
KEY_ACTION1 = pygame.K_F1


class KBReset:
    def __init__(self):
        pygame.init()
        self._screen = pygame.display.set_mode((800, 800))
        self._set_color(NORMAL)
        self._saving = False
        self._paused = False

    def update(self) -> str:
        pressed_last = self._get_pressed()

        # update and return state
        if KEY_QUIT_RECORDING in pressed_last:
            self._set_color(RED)
            self._saving = False
            return "normal"
        if KEY_PAUSE in pressed_last:
            if self._paused == False:
                print("--------------paused----------------")
                self._paused = True
                self._set_color(YELLOW)
                return "pause"
            else:
                print("--------------resumed---------------")
                self._paused = False
                self._set_color(NORMAL)
                return "normal"
        if KEY_START in pressed_last:
            self._set_color(GREEN)
            self._saving = True
            return "start"
        if KEY_ACTION1 in pressed_last:
            self._set_color(BLUE)
            return "action1"
        
        # some state outputs must be maintained if no key pressed
        if self._paused:
            self._set_color(YELLOW)
            return "pause"
        if self._saving:
            self._set_color(GREEN)
            return "save"

        # default state
        self._set_color(NORMAL)
        return "normal"

    def _get_pressed(self):
        pressed = []
        pygame.event.pump()
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                pressed.append(event.key)
        return pressed

    def _set_color(self, color):
        self._screen.fill(color)
        pygame.display.flip()


def main():
    kb = KBReset()
    while True:
        state = kb.update()
        if state == "start":
            print("start")


if __name__ == "__main__":
    main()
