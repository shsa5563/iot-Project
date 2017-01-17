/*
 * Circular_Buffer.c
 *
 *  Created on: 30-Oct-2016
 *      Author: Satyanarayana
 */


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "circular_Buffer.h"

//check if buffer is full
buffer_status buff_full(circular_buff_t *circ_buff)
{
    if (circ_buff->num_items == circ_buff->length)
    {
        return buffer_full;
    }
    else
    {
        return buffer_not_full;
    }

}
//allocate memory to buffer
int8_t* allocate_memory(circular_buff_t *circ_buff)
{
	circ_buff->buff= malloc(sizeof(circular_buff_t));
    return circ_buff->buff;
}

//free the memory allocated to buffer
void free_buff(circular_buff_t *circ_buff)
{
	free(circ_buff);
}

//check if buffer is empty
buffer_status buff_empty(circular_buff_t *circ_buff)
{
    if(circ_buff->num_items==0)
    {
        return buffer_empty;
    }
    else
    {
        return buffer_not_empty;
    }
}

//add item to buffer
buffer_status put_item_tobuffer(circular_buff_t *circ_buff, int8_t data)
{

    buffer_status _bufferstatus;
    _bufferstatus=buff_full(circ_buff);
    if(_bufferstatus==buffer_not_full)
    {
        if(circ_buff->head==(circ_buff->buff)+(circ_buff->length)-1)
        {
          circ_buff->head=circ_buff->buff;
          circ_buff->head++;
        *(circ_buff->head)=data;
        if(!(circ_buff->num_items!=MAX_LEN))
        (circ_buff->num_items)++;
        return  success_item_add_wraparound_override;
        }
        else
        {
            circ_buff->head++;
            *(circ_buff->head)=data;
            (circ_buff->num_items)++;
            return  success_item_add;
        }
    }
    else if(_bufferstatus==buffer_full)
    {
      return buffer_full;
    }

}

//read data from the buffer and remove the content saved in the buffer
int8_t read_delete_item_frombuffer(circular_buff_t *circ_buff)
{
    buffer_status _bufferstatus;
	int8_t data;
    _bufferstatus=buff_empty(circ_buff);
    if(_bufferstatus==buffer_not_empty)
    {

        if(circ_buff->tail==((circ_buff->buff)+(circ_buff->length))-1)
        {

        circ_buff->tail=circ_buff->buff;
        (circ_buff->tail)++;
        data=*(circ_buff->tail);
        *(circ_buff->tail)=0;
        (circ_buff->num_items)--;
        return data;
        }
        else
        {
            (circ_buff->tail)++;
            data=*(circ_buff->tail);
            *(circ_buff->tail)=0;
            (circ_buff->num_items)--;
            return data;
        }
    }
    else if(_bufferstatus==buffer_empty)
    {
      return -1;
    }
}
