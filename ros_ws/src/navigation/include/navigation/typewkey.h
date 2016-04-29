/**
 *  @file typeWkey.h
 *  @author shae, CS5201 Section A
 *  @date Apr 15, 2016
 *  @brief Description:
 *  @details Details:
 */

#ifndef TYPEWKEY_H_
#define TYPEWKEY_H_

template <typename T>
struct TypeWkey
{
  public:
    T m_data;
    TypeWkey()
    {

    }
    TypeWkey(const TypeWkey<T>& typewkey)
    {
      *this = typewkey;
    }

    TypeWkey(const T& t)
    {
      m_data = t;
    }

    const TypeWkey<T>& operator =(const T& data)
    {
      m_data = data;
      return *this;
    }

    const TypeWkey<T>& operator =( const TypeWkey<T>& typewkey)
    {
      m_data = typewkey.m_data;
      return *this;
    }


    int getKey() const
    {
      return m_data;
    }
};

template <typename T>
bool operator == (const TypeWkey<T>& lhs, const TypeWkey<T>& rhs)
{
  return lhs.getKey() == rhs.getKey();
}

template <typename T>
bool operator != (const TypeWkey<T>& lhs, const TypeWkey<T>& rhs)
{
  return !(lhs == rhs);
}


#endif /* TYPEWKEY_H_ */
