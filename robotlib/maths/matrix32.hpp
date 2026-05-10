/****************************************************************************
 *                       MARIX_F32_H
 *
 *       - Created by Sagar Chaudhary on Nov 13, 2023
 *
 *       This matrixf32 library is dependent on arm_math library.
 *
 ****************************************************************************/

/*
    Dynamic memory allocation is avoided.
    Template struct is used, so wisely use only few sizes of matrix otherwise it will consume lots of memory.
    No exeption is handled, instead used static assertion.
    Assign or copy data carefully especialy from where array of data through pointer is used to do.
    constructor = default is avoided, otherwise one class pointer will point other class data.
*/

#ifndef __MATRIX32_H__
#define __MATRIX32_H__

#include "arm_math.h"

template <uint16_t numRows, uint16_t numCols>
class Matrix32
{
public:
    float32_t data[numRows * numCols];
    arm_matrix_instance_f32 arm_mat;

    Matrix32() noexcept
    {
        arm_mat_init_f32(&arm_mat, numRows, numCols, data);
    }

    Matrix32(const Matrix32 &other) noexcept
    {
        arm_mat_init_f32(&arm_mat, numRows, numCols, data);
        memcpy(arm_mat.pData, other.arm_mat.pData, numRows * numCols * 4);
    }
    

    // template <typename... Args>
    // Matrix32(Args... values)
    //     : data{static_cast<float32_t>(values)...}
    // {
    //     static_assert(sizeof...(values) == numRows * numCols,
    //                   "Incorrect number of values. If there is static_cast error just above this line, then it is probably be due to mismatched size of matrix passed to copy or assign.");
    //     arm_mat_init_f32(&arm_mat, numRows, numCols, data);

    //     printf("template\n");
    // }

    Matrix32(const float32_t *pSrc) noexcept
    {
        arm_mat_init_f32(&arm_mat, numRows, numCols, data);
        memcpy(arm_mat.pData, pSrc, numRows * numCols * 4);
    }

    Matrix32 &operator=(const Matrix32 &other) noexcept
    {
        memcpy(arm_mat.pData, other.arm_mat.pData, numRows * numCols * 4);
        return *this;
    }

    Matrix32 &operator=(float32_t *pSrc) noexcept
    {
        memcpy(arm_mat.pData, pSrc, numRows * numCols * 4);
        return *this;
    }

    Matrix32 operator+(const Matrix32 &rhs) noexcept const
    {
        Matrix32 dst;
        arm_mat_add_f32(&this->arm_mat, &rhs.arm_mat, &dst.arm_mat);
        return dst;
    }

    Matrix32 operator-(const Matrix32 &rhs) noexcept const
    {
        Matrix32 dst;
        arm_mat_sub_f32(&this->arm_mat, &rhs.arm_mat, &dst.arm_mat);
        return dst;
    }

    template <uint16_t numAny>
    Matrix32<numRows, numAny> operator*(const Matrix32<numCols, numAny> &rhs) noexcept const
    {
        Matrix32<numRows, numAny> dst;
        arm_mat_mult_f32(&this->arm_mat, &rhs.arm_mat, &dst.arm_mat);
        return dst;
    }

    Matrix32<numCols, numRows> trans() noexcept const
    {
        Matrix32<numCols, numRows> dst;
        arm_mat_trans_f32(&this->arm_mat, &dst.arm_mat);
        return dst;
    }

    Matrix32 inverse() noexcept const
    {
        static_assert(numRows == numCols,
                      "Inverse of rectangular matrix does not exists. Matrix must be square to have inverse.");
        Matrix32 dst;
        arm_status status = arm_mat_inverse_f32(&this->arm_mat, &dst.arm_mat);
        if (status == ARM_MATH_SINGULAR)
        {
            printf("EXCEPTION:: MATRIX INVERSION FAILED !!\n");
        }

        return dst;
    }

    void fill(const float32_t &value) noexcept
    {
        arm_fill_f32(value, arm_mat.pData, numRows * numCols);
    }

    void setIdentity() noexcept
    {
        static_assert(numRows == numCols, "Only square matrix can be identity matrix");
        for (uint16_t i = 0; i < numRows; ++i)
        {
            for (uint16_t j = 0; j < numCols; ++j)
            {
                arm_mat.pData[i * numCols + j] = (i == j) ? 1.0f : 0.0f;
            }
        }
    }

    void printData() noexcept const
    {
        for (uint16_t i = 0; i < numRows; ++i)
        {
            for (uint16_t j = 0; j < numCols; ++j)
            {
                printf("%f\t", arm_mat.pData[i * numCols + j]);
            }
            printf("\n");
        }
    }

    ~Matrix32() = default;
};

#endif
