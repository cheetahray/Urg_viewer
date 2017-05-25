#ifndef HRK_RING_BUFFER_HPP
#define HRK_RING_BUFFER_HPP

/*!
  \file
  \brief リングバッファ

  \author Satofumi KAMIMURA

  $Id$
*/

#include <cstddef>
#include <deque>


namespace hrk
{
    //! リングバッファ
    template <class T>
    class Ring_buffer
    {
    public:
        Ring_buffer(void)
        {
        }


        //! バッファサイズの取得
        size_t size(void) const
        {
            return ring_buffer_.size();
        }


        /*!
          \brief バッファが空か

          \retval true データなし
          \retval false データあり
        */
        bool empty(void) const
        {
            return ring_buffer_.empty();
        }


        /*!
          \brief データの格納

          \param[in] data データ
          \param[in] size データ個数

          \return 格納したデータ個数
        */
        size_t push(const T* data, size_t size)
        {
            const T* last_p = data + size;
            ring_buffer_.insert(ring_buffer_.end(), data, last_p);
            return size;
        }


        /*!
          \brief データの取り出し

          \param[out] data データ取り出し用バッファ
          \param[in] size 取り出すデータの最大個数

          \return 取り出したデータ個数
        */
        size_t pop(T* data, size_t size)
        {
            size_t n = std::min(size, ring_buffer_.size());
            std::copy(ring_buffer_.begin(), ring_buffer_.begin() + n, data);
            ring_buffer_.erase(ring_buffer_.begin(), ring_buffer_.begin() + n);
            return n;
        }


        /*!
          \brief データの書き戻し

          \param[in] ch 書き戻すデータ
        */
        void ungetc(const T ch)
        {
            ring_buffer_.push_front(ch);
        }


        //! 格納データのクリア
        void clear(void)
        {
            ring_buffer_.clear();
        }


    private:
        Ring_buffer(const Ring_buffer& rhs);
        Ring_buffer& operator = (const Ring_buffer& rhs);

        std::deque<T> ring_buffer_;
    };
}

#endif
