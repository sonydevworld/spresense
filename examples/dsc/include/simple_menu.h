/****************************************************************************
 * examples/dsc/include/simple_menu.h
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

#ifndef __EXAMPLES_DSC_INCLUDE_SIMPLE_MENU_H__
#define __EXAMPLES_DSC_INCLUDE_SIMPLE_MENU_H__

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MENU_EXEC_DONE    (0)
#define MENU_EXEC_SUBMENU (1)
#define MENU_EXEC_BACK    (2)
#define MENU_EXEC_END     (3)

#define MENU_KEY_NONE   (0)
#define MENU_KEY_SELECT (1)
#define MENU_KEY_UP     (2)
#define MENU_KEY_DOWN   (3)
#define MENU_KEY_LEFT   (4)
#define MENU_KEY_RIGHT  (5)

/****************************************************************************
 * Public types
 ****************************************************************************/

typedef int (*menu_execute_func_t)(int *value);

/****************************************************************************
 * Public Classes
 ****************************************************************************/

class menu;

class menu_item
{
  protected:
    const char *item_name;
    int item_value;
    menu_item *next_item;
    menu_item *prev_item;

    menu_execute_func_t exec_func;

  public:
    menu_item(const char *name, int init_value);
    virtual ~menu_item();

    void insert_infront(menu_item *i);
    menu_item *operator [](int ofst);
    int get_value();
    void set_execfunc(menu_execute_func_t e);

    virtual const char *get_itemname();
    virtual const char *get_valuename();
    virtual int exec();
};

class endmenu_item : public menu_item
{
  public:
    endmenu_item(const char *name) : menu_item(name, 0)
    {
    };

    virtual ~endmenu_item()
    {
    };

    virtual int exec()
    {
      return MENU_EXEC_END;
    };
};

class submenu_item : public menu_item
{
  private:
    menu *submenu;

  public:
    submenu_item(const char *name);
    virtual ~submenu_item();

    int set_submenu(menu *s);
    menu *get_submenu();

    virtual int exec();
};

class back_item : public submenu_item
{
  public:
    back_item();
    virtual ~back_item();

    virtual const char *get_itemname();
    virtual int exec();
};

class menu
{
  private:
    const char *menu_title;
    int menu_num;
    menu_item *items;
    back_item back;

    int item_pos;
    int display_pos;

  public:
    menu(const char *ttl);
    ~menu();

    const char *get_menutitle();
    const char *get_itemname(int ofst);

    int get_displaypos();
    int get_itempos();
    int get_menunum();

    menu_item *get_menuitem(int i);
    void set_menuitem(menu_item *i);

    menu *get_submenu();
    void set_backmenu(menu *m);

    void reset_top();
    void reset_bottom(int lines);

    void go_next(int lines);
    void go_prev(int lines);

    int exec_item();
};

class display_menu
{
  private:
    int disp_lines;

  public:
    display_menu(int lines);
    virtual ~display_menu();

    int lines();

    virtual void draw_title(menu *m) = 0;
    virtual void draw_tail(menu *m);
    virtual void draw_menuitem(menu_item *itm, int line, bool sel) = 0;
};

class menu_system
{
  private:
    display_menu  *drawer;
    menu *current_menu;

  public:
    menu_system();
    virtual ~menu_system();

    void set_drawer(display_menu *dsp);
    void set_menu(menu *m);

    int input_key(int key);
    void draw_menu();
    virtual void set_canvas(void *);
};

#endif  /* __EXAMPLES_DSC_INCLUDE_SIMPLE_MENU_H__ */
