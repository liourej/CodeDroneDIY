#ifndef SINGLETON_H_
#define SINGLETON_H_

template <typename Class> class SingletonSimple {
  protected:
    // Constructeur/destructeur
    SingletonSimple() {
    }
    ~SingletonSimple() {
    }

  public:
    // Public interface with one argument
    template <typename... _Args> static Class *GetInstance(_Args... _arg) {
        if (_singletonSimple == nullptr) {
            _singletonSimple = new Class(_arg...);
        }
        return (static_cast<Class *>(_singletonSimple));
    }

    static void kill() {
        if (nullptr != _singletonSimple) {
            delete _singletonSimple;
            _singletonSimple = nullptr;
        }
    }

  private:
    // Unique instance
    static Class *_singletonSimple;
};

template <typename Class>
Class *SingletonSimple<Class>::_singletonSimple = nullptr;

template <typename childClass, typename motherClass> class Singleton : public motherClass {
  protected:
    // Constructeur/destructeur
    Singleton() {
    }
    ~Singleton() {
    }

  public:
    // Public interface with one argument
    template <typename... _Args> static childClass *GetInstance(_Args... _arg) {
        if (_singleton == nullptr) {
            _singleton = new childClass(_arg...);
        }
        return (static_cast<childClass *>(_singleton));
    }

    static void kill() {
        if (nullptr != _singleton) {
            delete _singleton;
            _singleton = nullptr;
        }
    }

  private:
    // Unique instance
    static childClass *_singleton;
};

template <typename childClass, typename motherClass>
childClass *Singleton<childClass, motherClass>::_singleton = nullptr;
#endif // SINGLETON_H_