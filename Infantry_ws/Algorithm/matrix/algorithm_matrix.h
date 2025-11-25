#ifndef _ALGORITHN_MATRIX_H_
#define _ALGORITHN_MATRIX_H_

#ifdef __cplusplus
extern "C" {
#endif
	
#include <cassert>
	
#ifdef __cplusplus
	
}
#endif

template <int Rows, int Cols> class Matrix {

public:
  Matrix() {}

  template <class... Args> Matrix(Args... args) {
    assert((sizeof...(args) == (Rows * Cols)));
    const float datas[Rows * Cols] = {static_cast<float>(args)...};
    for (int i = 0; i < Rows * Cols; i++) {
      data_[i] = datas[i];
    }
  }

  template <class... Args> static Matrix<Rows, Cols> concat_from(Args... args) {
    Matrix<Rows, Cols> m;
    assert(sizeof...(args) == Cols);

    Matrix<Rows, 1> vs[Cols] = {args...};

    for (int col = 0; col < Cols; col++) {
      for (int row = 0; row < Rows; row++) {

        m.data_[row * Cols + col] = vs[col](row, 0);
      }
    }

    return m;
  }


  void operator=(const Matrix &m)
  {
    for (int i = 0; i < Rows * Cols; i++) {
      this->data_[i] = m.data_[i];
    }
  }

  float operator()(int row, int col) const { return data_[row * Cols + col]; }

  ~Matrix() {}

private:
  float data_[Rows * Cols];
};

#endif // !_ALGORITHN_MATRIX_H_
