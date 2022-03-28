/****************************************************************************
 * examples/dsc/utils/simple_menu.cxx
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
 * Included Files
 ****************************************************************************/

#include <stdlib.h>
#include "simple_menu.h"

/****************************************************************************
 * Class Methods
 ****************************************************************************/

//* menu_item class methods **************************************************

menu_item::menu_item(const char *name, int init_value) :
      item_name(name), item_value(init_value),
      next_item(NULL), prev_item(NULL), exec_func(NULL)
{
}

menu_item::~menu_item()
{
}

void menu_item::insert_infront(menu_item *i)
{
  if (prev_item)
    {
      prev_item->next_item = i;
      i->prev_item = prev_item;

      i->next_item = this;
      prev_item = i;
    }

  i->next_item = this;
  prev_item = i;
}

menu_item *menu_item::operator [](int ofst)
{
  menu_item *ret = this;
  while (ofst-- && ret)
    {
      ret = ret->next_item;
    }

  return ret;
}

const char *menu_item::get_itemname()
{
  return item_name;
}

int menu_item::get_value()
{
  return item_value;
}

const char *menu_item::get_valuename()
{
  return NULL;
}

void menu_item::set_execfunc(menu_execute_func_t e)
{
  exec_func = e;
}

int menu_item::exec()
{
  return exec_func ? exec_func(&item_value) : MENU_EXEC_DONE;
}

//* submenu_item class methods ***********************************************

submenu_item::submenu_item(const char *name) :
    menu_item(name, 0), submenu(NULL)
{
}

submenu_item::~submenu_item()
{
}

int submenu_item::set_submenu(menu *s)
{
  int ret = submenu == NULL ? 1 : 0;
  submenu = s;
  return ret;
}

menu *submenu_item::get_submenu()
{
  return submenu;
}

int submenu_item::exec()
{
  return MENU_EXEC_SUBMENU;
}

//* back_item class methods **************************************************

back_item::back_item() : submenu_item("Back")
{
}

back_item::~back_item()
{
}

const char *back_item::get_itemname()
{
  return get_submenu() ? item_name : NULL;
}

int back_item::exec()
{
  return MENU_EXEC_BACK;
}

//* menu class methods *******************************************************

menu::menu(const char *ttl) :
    menu_title(ttl), menu_num(0), items(NULL), item_pos(0), display_pos(0)
{
}

menu::~menu()
{
}

const char *menu::get_menutitle()
{
  return menu_title;
}

const char *menu::get_itemname(int ofst)
{
  return menu_title;
}

int menu::get_displaypos()
{
  return display_pos;
}

int menu::get_itempos()
{
  return item_pos;
}

int menu::get_menunum()
{
  return menu_num;
}

menu_item *menu::get_menuitem(int i)
{
  return (*items)[i];
}

void menu::set_menuitem(menu_item *i)
{
  if (!items)
    {
      items = i;
    }

  back.insert_infront(i);
  menu_num++;
}

void menu::set_backmenu(menu *m)
{
  menu_num += back.set_submenu(m);
}

void menu::reset_top()
{
  item_pos = display_pos = 0;
}

void menu::go_next(int lines)
{
  item_pos++;
  if (item_pos >= menu_num)
    {
      reset_top();
    }
  else if (item_pos >= (display_pos + lines))
    {
      display_pos = item_pos - lines + 1;
      if ((display_pos + lines) >= menu_num)
        {
          display_pos = menu_num - lines;
        }
    }
}

void menu::reset_bottom(int lines)
{
  item_pos = menu_num - 1;
  display_pos = menu_num - lines;
  if (display_pos < 0)
    {
      display_pos = 0;
    }
}

void menu::go_prev(int lines)
{
  item_pos--;
  if (item_pos < 0)
    {
      reset_bottom(lines);
    }
  else if (item_pos < display_pos)
    {
      display_pos = item_pos;
    }
}

int menu::exec_item()
{
  return (*items)[item_pos]->exec();
}

menu *menu::get_submenu()
{
  return ((submenu_item *)(*items)[item_pos])->get_submenu();
}

//* display_menu class methods ***********************************************

display_menu::display_menu(int lines) : disp_lines(lines)
{
}

display_menu::~display_menu()
{
}

int display_menu::lines()
{
  return disp_lines;
}

void display_menu::draw_tail(menu *m)
{
}

//* menu_system class methods ************************************************

menu_system::menu_system(): drawer(NULL), current_menu(NULL)
{
}

menu_system::~menu_system()
{
}

void menu_system::set_drawer(display_menu *dsp)
{
  drawer = dsp;
}

void menu_system::set_menu(menu *m)
{
  current_menu = m;
}

int menu_system::input_key(int key)
{
  int ret = MENU_EXEC_DONE;
  menu *tmp;

  switch (key)
    {
      case MENU_KEY_SELECT:
        ret = current_menu->exec_item();
        if (ret == MENU_EXEC_SUBMENU || ret == MENU_EXEC_BACK)
          {
            tmp = current_menu->get_submenu();
            if (ret == MENU_EXEC_SUBMENU)
              {
                tmp->set_backmenu(current_menu);
                tmp->reset_top();
              }
  
            current_menu = tmp;
          }
      break;

      case MENU_KEY_UP:
        current_menu->go_prev(drawer->lines());
      break;

      case MENU_KEY_DOWN:
        current_menu->go_next(drawer->lines());
      break;
    }

  return ret;
}

void menu_system::draw_menu()
{
  int i;
  int disp_pos;
  menu_item *item;

  drawer->draw_title(current_menu);

  disp_pos = current_menu->get_displaypos();

  for (i = disp_pos; i < disp_pos + drawer->lines(); i++)
    {
      item = current_menu->get_menuitem(i);
      if (i < current_menu->get_menunum() && item->get_itemname())
        {
          drawer->draw_menuitem(item, i - disp_pos,
                                i == current_menu->get_itempos());
        }
    }

  drawer->draw_tail(current_menu);
}

void menu_system::set_canvas(void *)
{
  /* Do nothing. */
}
