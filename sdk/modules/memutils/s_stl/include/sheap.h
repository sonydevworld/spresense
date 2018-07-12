/****************************************************************************
 * modules/memutils/s_stl/include/sheap.h
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

#ifndef SHEAP_H_INCLUDED
#define SHEAP_H_INCLUDED

#include "node.h"

__STL_BEGIN_NAMESPACE

template<class T> class SHeap{

private:
	Node<T>* tree_top;

public:

	SHeap():tree_top(NULL_NODE){}

	bool add(Node<T>* new_node){
		if(empty()) {
			tree_top = new_node;
			return TRUE;
		}
		tree_top = marge(tree_top, new_node);
		return TRUE;
	}

	const T& top(void) const {
		if(empty()) { _fatal; }
		return tree_top->data;
	}

	Node<T>* remove(void){
		if(empty()) { return NULL_NODE; }

		Node<T>* free_node = tree_top;
		tree_top = marge(tree_top->left,tree_top->right);
		return free_node;
	}

	bool empty(void) const {
		return (tree_top == NULL_NODE);
	}

	void clear(void){
		tree_top = NULL_NODE;
	}

private:
	Node<T>* marge(Node<T>* tr1,Node<T>* tr2){
		if(!tr1) return tr2;
		if(!tr2) return tr1;

		if(*tr1 > *tr2)
			return marge(tr2,tr1);

		Node<T>* child = tr1->left;
		if(child){
			tr1->left = marge(tr1->right,tr2);
			tr1->right = child;
		}else{
			tr1->left = tr2;
		}
		return tr1;
	}
};

__STL_END_NAMESPACE

#endif // SHEAP_H_INCLUDED

