//
// Created by nathan.l.gordon on 2019-04-05.
//

#ifndef TMC_2130_PIMPL_IMPL_H
#define TMC_2130_PIMPL_IMPL_H

#include "pimpl.h"

template<typename T>
pimpl<T>::pimpl() : m { new T{} } {}

template<typename T>
template<typename ...Args>
pimpl<T>::pimpl(Args&& ...args) : m { new T {std::forward<Args>(args)...}} {}

template<typename T>
pimpl<T>::~pimpl() = default;

template<typename T>
T* pimpl<T>::operator->() { return m.get(); }

template<typename T>
T *pimpl<T>::operator->() const { return m.get(); }

template<typename T>
T& pimpl<T>::operator*() { return *m.get(); }

template<typename T>
T &pimpl<T>::operator*() const { return *m.get(); }


template<typename T>
pimpl_shared<T>::pimpl_shared() : m { new T{} } {}

template<typename T>
pimpl_shared<T>::pimpl_shared(std::shared_ptr<T> &input) : m {input } { }

template<typename T>
template<typename ...Args>
pimpl_shared<T>::pimpl_shared(Args&& ...args) : m { new T {std::forward<Args>(args)...}} {}

template<typename T>
pimpl_shared<T>::~pimpl_shared() = default;

template<typename T>
T* pimpl_shared<T>::operator->() { return m.get(); }

template<typename T>
T *pimpl_shared<T>::operator->() const { return m.get(); }

template<typename T>
T& pimpl_shared<T>::operator*() { return *m.get(); }

template<typename T>
T &pimpl_shared<T>::operator*() const { return *m.get(); }

template<typename T>
std::shared_ptr<T> pimpl_shared<T>::share() const {
    return m;
}


#endif //TMC_2130_PIMPL_IMPL_H
