#ifndef __reference_object_h__
#define __reference_object_h__

class ReferenceCountable_BADLY_BROKEN {
public:
    ReferenceCountable_BADLY_BROKEN() : m_counter(0) {}

    //the method is virtual in case a derived class needs to
    //do something special while performing the operation.
    //Also, the method is const so that we could ref/unref an object
    //that is pointed at by a const pointer.
    virtual int  ref() const { return ++m_counter; }

    //the method is virtual in case a derived class needs to
    //do something special while performing the operation.
    //The method will destroy the object once the counter reaches 0.
    //Also, the method is const so that we could ref/unref an object
    //that is pointed at by a const pointer.
    virtual int  unref() const 
    { 
        --m_counter;
        if(m_counter<=0)
        {
            int ret = m_counter;
            //YES, the object is being des
            delete this;
            return ret;
        }
        return m_counter;
    }

    int getRef() const { return m_counter; }

protected:

    //yes, protected and virtual.
    //Virtual because of inheritance.
    //Protected because of reference counting - a user should only
    //be able to destroy the object via the unref call. Otherwise,
    //the entire reference counting system falls apart.
    virtual ~ReferenceCountable_BADLY_BROKEN() {}

private:

    //mutable because ref/unref are const methods.
    mutable int m_counter;
};


#endif
