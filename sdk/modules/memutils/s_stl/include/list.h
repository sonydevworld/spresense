/****************************************************************************
 * modules/memutils/s_stl/include/list.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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

#ifndef LIST_H_INCLUDED
#define LIST_H_INCLUDED

#include "s_stl_config.h"
#include "node.h"

__STL_BEGIN_NAMESPACE

/*-------------------------------------------------------------------*/
template<class T> class OneWayList {

private:

	Node<T>* list_top;

public:
	OneWayList() : list_top(NULL_NODE){}

	bool add(Node<T>* new_node)
	{
		Node<T>* list_p;

		if(empty()) {
			list_top = new_node;
			list_top->right = NULL_NODE;
			return TRUE;
		} else {
			if(*list_top > *new_node){
				new_node->right = list_top;
				list_top = new_node;
				return TRUE;
			}
			for(list_p = list_top;list_p->right != NULL_NODE;
			    list_p = list_p->right){
				if((*list_p->right) > *new_node){
					new_node->right = list_p->right;
					list_p->right = new_node;
					return TRUE;
				}
			}
		}
		list_p->right =  new_node;
		new_node->right = NULL_NODE;
		return TRUE;
	}

	Node<T>* remove(void){
		if(empty()) {
			_fatal;
			return NULL_NODE;
		}
		Node<T>* free_node = list_top;
		list_top = list_top->right;
		return free_node;
	}

	const T& top(void) const {
		if(empty()) { _fatal; }
		return list_top->data;
	}

	void clear(void){ list_top = NULL_NODE; }

	bool empty(void) const { return (list_top == NULL_NODE); }

};

__STL_END_NAMESPACE

#endif // LIST_H_INCLUDED

