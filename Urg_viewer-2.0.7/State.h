#ifndef STATE_H
#define STATE_H

/*!
  \file
  \brief 状態遷移の管理

  \author Satofumi Kamimura

  $Id$
*/

class State
{
public:
    typedef enum {
        Not_connected,
        Viewing,
        Playing,
        Recording,
    } state_t;


    virtual ~State(void)
    {
    }


    virtual void set_state(state_t state) = 0;
};

#endif
