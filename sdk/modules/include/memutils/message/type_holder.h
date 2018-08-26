/****************************************************************************
 * modules/include/memutils/message/type_holder.h
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

#ifndef TYPE_HOLDER_H_INCLUDED
#define TYPE_HOLDER_H_INCLUDED

#include <stdio.h>		/* printf, size_t */
#include "memutils/common_utils/common_types.h"
#include "memutils/common_utils/common_assert.h"
#include "memutils/message/AssertInfo.h"

/* Returns the class identifier corresponding to type. */

#define GET_TYPE_ID(type)	(&TypeHolder<type>::type_size)

template<typename T> class TypeHolder;

/*****************************************************************
 * Base class for processing TypeHolder<T> class in common
 *****************************************************************/
class TypeHolderBase {
public:
	typedef size_t (*id_t)(); /* Type identifier. */

	virtual ~TypeHolderBase() {}

	template<typename T>
	bool is_type() const { return this->id() == GET_TYPE_ID(T); }

	template<typename T>
	T& get() {
		D_ASSERT2(this->is_type<T>(),
			AssertParamLog(AssertIdTypeUnmatch, (uint32_t)this->id(), (uint32_t)GET_TYPE_ID(T)));
		return static_cast<TypeHolder<T>*>(this)->get();
	}

	template<typename T>
	const T& get() const {
		D_ASSERT2(this->is_type<T>(),
			AssertParamLog(AssertIdTypeUnmatch, (uint32_t)this->id(), (uint32_t)GET_TYPE_ID(T)));
		return static_cast<const TypeHolder<T>*>(this)->get();
	}

	template<typename T>
	T& get_any(bool size_check = true) {
		if (size_check) {
			D_ASSERT2(sizeof(T) <= this->size(),
				AssertParamLog(AssertIdSizeError, sizeof(T), this->size()));
		}
		return static_cast<TypeHolder<T>*>(this)->get();
	}

	template<typename T>
	const T& get_any(bool size_check = true) const {
		if (size_check) {
			D_ASSERT2(sizeof(T) <= this->size(),
				AssertParamLog(AssertIdSizeError, sizeof(T), this->size()));
		}
		return static_cast<const TypeHolder<T>*>(this)->get();
	}

	size_t size() const { return this->id()(); }

	void dump() const {
		size_t sz = this->size();
		const unsigned char* data = &get_any<unsigned char>();
		printf("size:%u, id:%p, data: ", sz, this->id());
		for (size_t i = 0; i < MIN(sz, 16); ++i) {
			printf("%02x ", data[i]);
		}
		printf("\n");
	}

	virtual id_t id() const = 0;

#ifdef USE_TYPE_HOLDER_ACTION_API
  /* Subclass of TypeHolder class, overriding it if necessary. */

	virtual void* action(void*) { return 0; }
	virtual void* action(void*) const { return 0; }
#endif
}; /* TypeBase */

/*****************************************************************
 * Class that holds an instance of an arbitrary type and type information
 * To uniquely identify the storage type,
 * the template argument must be of type only
 *****************************************************************/
template<typename T>
class TypeHolder : public TypeHolderBase {
public:
  /* A unique function is generated for each class. */

	static size_t type_size() { return sizeof(T); }

	TypeHolder() : m_held() {}
	explicit TypeHolder(const T& data) : m_held(data) {}

	T& get() { return m_held; }
	const T& get() const { return m_held; }

  /* Use the address of a unique function for each class
   * for class identification.
   */

	virtual id_t id() const { return &type_size; }

private:
	T	m_held;
}; /* TypeHolder */

#endif /* TYPE_HOLDER_H_INCLUDED */
