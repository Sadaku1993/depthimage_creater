#include <math.h>

//平面の定義
class Plane {	//ax+by+cz+d=0
public:
	double a,b,c,d;
	
	Plane(){}
	Plane(double a,double b,double c,double d){ this->a = a; this->b = b; this->c = c; this->d = d; }
};

//ベクトルの定義
class Vector3D{
public:
	double x,y,z;
	
	Vector3D(){}
	Vector3D( double x, double y, double z) {this->x = x; this->y = y; this->z = z; }

	//ベクトル引き算( this - v )
	Vector3D operator - ( const Vector3D& v ) const { return Vector3D( x - v.x, y - v.y, z - v.z ); }
	//ベクトル外積( this × vr )
	Vector3D operator * ( const Vector3D& vr ) const { return Vector3D( (y * vr.z) - (z * vr.y), (z * vr.x) - (x * vr.z), (x * vr.y) - (y * vr.x) ); }
	//自身を単位ベクトルにする
	void Normalize() {
		double length = pow( ( x * x ) + ( y * y ) + ( z * z ), 0.5 );//ベクトルの長さ
		x /= length;
		y /= length;
		z /= length;
	}
};
//頂点の定義(ベクトルと同じ)
#define Vertex3D Vector3D

//頂点abcで作られたポリゴンから法線を計算する。
Vector3D CreatePolygonNormal( Vertex3D a, Vertex3D b, Vertex3D c ) {

	Vector3D ab = b - a;
	Vector3D bc = c - b;

	Vector3D normal = ab * bc;	//ab bcの外積
	normal.Normalize();//単位ベクトルにする

	return normal;
}

//ひとつの頂点と法線ベクトルから平面を作成する
Plane CreatePlaneFromPointNormal( Vertex3D p, Vector3D normal )//※normalは単位ベクトルであること
{
	//pとnormalを内積
	double d = p.x * normal.x + p.y * normal.y + p.z * normal.z;

	return Plane( normal.x, normal.y, normal.z, d );
}

//ポリゴンから平面を作成する
Plane CreatePlaneFromPolygon( Vertex3D a, Vertex3D b, Vertex3D c )//※abcは同一でないこと
{	
	//ポリゴンの法線を計算する
	Vector3D normal = CreatePolygonNormal(a,b,c);
	
	//ポリゴンのどれかひとつの頂点と法線ベクトルから平面を作成する
	return CreatePlaneFromPointNormal( a, normal );
}

