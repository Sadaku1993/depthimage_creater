#include <math.h>

//ベクトルの定義と各種計算
struct VectorXD{
    double x;
    double y;
    double z;
};

//頂点の定義(ベクトルと同じ)
#define VertexXD VectorXD

//ベクトル引き算(a-b)
VectorXD sub_vector( const VectorXD& a, const VectorXD& b )
{
    VectorXD ret;
    ret.x = a.x - b.x;
    ret.y = a.y - b.y;
    ret.z = a.z - b.z;
    return ret;
}

//ベクトル外積( vl × vr )
VectorXD cross_product( const VectorXD& vl, const VectorXD& vr )
{
    VectorXD ret;
    ret.x = vl.y * vr.z - vl.z * vr.y;
    ret.y = vl.z * vr.x - vl.x * vr.z;
    ret.z = vl.x * vr.y - vl.y * vr.x;

    return ret;
}

//ベクトル内積
double dot_product( const VectorXD& vl, const VectorXD vr) {
    return vl.x * vr.x + vl.y * vr.y + vl.z * vr.z;
}

// 三角形と点の当たり判定(３Dの場合)
// 戻り値    0:三角形の内側に点がある    1:三角形の外側に点がある
int hittest_point_polygon_3d( VertexXD A, VertexXD B, VertexXD C, VertexXD P ) {

    //点と三角形は同一平面上にあるものとしています。同一平面上に無い場合は正しい結果になりません
    //線上は外とみなします。
    //ABCが三角形かどうかのチェックは省略...
    
    VectorXD AB = sub_vector(B, A);
    VectorXD BP = sub_vector(P, B);

    VectorXD BC = sub_vector(C, B);
    VectorXD CP = sub_vector(P, C);

    VectorXD CA = sub_vector(A, C);
    VectorXD AP = sub_vector(P, A);

    VectorXD c1 = cross_product( AB, BP );
    VectorXD c2 = cross_product( BC, CP );
    VectorXD c3 = cross_product( CA, AP );


    //内積で順方向か逆方向か調べる
    double dot_12 = dot_product(c1, c2);
    double dot_13 = dot_product(c1, c3);

    if( dot_12 > 0 && dot_13 > 0 ) {
        //三角形の内側に点がある
        return 1;
    }

    //三角形の外側に点がある
    return 0;

}
