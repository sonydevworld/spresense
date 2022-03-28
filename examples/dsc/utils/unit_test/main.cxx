/****************************************************************************
 * dsc/utils/unit_test/main.cxx
 *
 *   Copyright 2022 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 *
 * This code is a test code for simple_menu.
 * simple_menu requires an instance of a class extends display_menu.
 *
 * The test menu hierarchy is 
 *   menu1 -+
 *          |
 *          +- item1_1
 *          +- item1_2 -- menu2 -+
 *          +- item1_3           +- item2_1
 *          +- item1_4           +- item2_2
 *          +- item1_5           +- Back
 *          +- item1_6
 *          +- item1_7 -- menu3 -+
 *          +- item1_8           +- item3_1
 *                               +- Back
 *
 * To Walk around in the menu
 *       'k'   : Up key
 *       'j'   : Down key
 *       space : select key
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>

#include <simple_menu.h>

/****************************************************************************
 * Private Classes
 ****************************************************************************/

class printf_display : public display_menu
{
  public:
    printf_display() : display_menu(3) {};

    virtual void draw_title(menu *m)
      {
        printf("[%s]\n\n", m->get_menutitle());
        if (m->get_displaypos() != 0)
          {
            printf("  more..\n");
          }
      };

    virtual void draw_menuitem(menu_item *itm, int line, bool sel)
      {
        printf("  %s%s", (sel) ? "-> " : "   ", itm->get_itemname());
        if (itm->get_valuename())
          {
            printf(" -- %s\n", itm->get_valuename());
          }
        else
          {
            printf("\n");
          }
      }

    virtual void draw_tail(menu *m)
      {
        if (m->get_menunum() > m->get_displaypos() + lines())
          {
            printf("  more..\n");
          }
        printf("\n");
      }
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static menu menu1("Top Menu");
static menu_item    item1_1("Item 1", 0);
static submenu_item item1_2("SubMenu");
static menu_item    item1_3("Item 3", 0);
static menu_item    item1_4("Item 4", 0);
static menu_item    item1_5("Item 5", 0);
static menu_item    item1_6("Item 6", 0);
static submenu_item item1_7("Sub2");
static menu_item    item1_8("Item 8", 0);

static menu menu2("Sub Menu");
static menu_item item2_1("Item A", 0);
static menu_item item2_2("Item B", 0);

static menu menu3("SubSub Menu");
static menu_item item3_1("Item--", 0);

static printf_display disp;
static menu_system sys;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void create_menus()
{
  menu1.set_menuitem(&item1_1);
  item1_2.set_submenu(&menu2);
  menu1.set_menuitem(&item1_2);
  menu1.set_menuitem(&item1_3);
  menu1.set_menuitem(&item1_4);
  menu1.set_menuitem(&item1_5);
  menu1.set_menuitem(&item1_6);
  item1_7.set_submenu(&menu3);
  menu1.set_menuitem(&item1_7);
  menu1.set_menuitem(&item1_8);

  menu2.set_menuitem(&item2_1);
  menu2.set_menuitem(&item2_2);

  menu3.set_menuitem(&item3_1);

  sys.set_drawer(&disp);
  sys.set_menu(&menu1);
}

static void set_nonblocking(struct termios *saved)
{
	struct termios settings;

	tcgetattr(0, saved);
	settings = *saved;

	settings.c_lflag &= ~(ECHO|ICANON);
	settings.c_cc[VTIME] = 0;
	settings.c_cc[VMIN] = 1;
	tcsetattr(0,TCSANOW,&settings);
	fcntl(0,F_SETFL,O_NONBLOCK);
}

static void store_setting(struct termios *saved)
{
	tcsetattr(0,TCSANOW,saved);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(void)
{
	struct termios saved_setting;
  int c;
  int key;
  bool is_running = true;

  set_nonblocking(&saved_setting);
  create_menus();

  sys.draw_menu();

  while (is_running)
    {
      key = MENU_KEY_NONE;
      c = getchar();
      if (c != EOF)
        {
          switch (c)
            {
              case 'k':
                key = MENU_KEY_UP;
                break;
              case 'j':
                key = MENU_KEY_DOWN;
                break;
              case ' ':
                key = MENU_KEY_SELECT;
                break;
              case 'q':
                is_running = false;
                break;
            }

          sys.input_key(key);
          if (key != MENU_KEY_NONE)
            {
              sys.draw_menu();
            }
        }
    }

  store_setting(&saved_setting);

  return 0;
}
