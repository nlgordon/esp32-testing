//
// Created by nathan.l.gordon on 2019-04-05.
//

#ifndef TMC_2130_PIMPL_H
#define TMC_2130_PIMPL_H
#include <memory>

template<typename T>
class pimpl {
private:
    std::unique_ptr<T> m;
public:
    pimpl();
    template<typename ...Args> explicit pimpl(Args&& ...);
    ~pimpl();
    T* operator->();
    T& operator*();
    T* operator->() const;
    T& operator*() const;
};

template<typename T>
class pimpl_shared {
private:
    std::shared_ptr<T> m;
public:
    pimpl_shared();
    pimpl_shared(std::shared_ptr<T> input);
    template<typename ...Args> explicit pimpl_shared(Args&& ...);
    ~pimpl_shared();
    T* operator->();
    T& operator*();
    T* operator->() const;
    T& operator*() const;
};

#endif //TMC_2130_PIMPL_H
