using UnityEngine;
using System.Collections;
using System.Numerics;
using Unity.VisualScripting;
using Matrix4x4 = UnityEngine.Matrix4x4;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia
	
	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;					// for collision

	private Vector3 gravity = new Vector3(0, -9.8f, 0);
	private Vector3[] vertices;
	private float mu = 0.5f;

	private float dv;

	// Use this for initialization
	void Start () 
	{		
		// 获取初始 Mesh 顶点信息
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		vertices = mesh.vertices;

		// m 单位质量，mass 总质量
		float m = 1;
		mass = 0;
		
		// 计算 inertia I
		for (int i = 0; i < vertices.Length; i++) 
		{
			mass += m;
			float diag = m * vertices[i].sqrMagnitude;
			I_ref[0, 0] += diag;
			I_ref[1, 1] += diag;
			I_ref[2, 2] += diag;
			I_ref[0, 0] -= m * vertices[i][0] * vertices[i][0];
			I_ref[0, 1] -= m * vertices[i][0] * vertices[i][1];
			I_ref[0, 2] -= m * vertices[i][0] * vertices[i][2];
			I_ref[1, 0] -= m * vertices[i][1] * vertices[i][0];
			I_ref[1, 1] -= m * vertices[i][1] * vertices[i][1];
			I_ref[1, 2] -= m * vertices[i][1] * vertices[i][2];
			I_ref[2, 0] -= m * vertices[i][2] * vertices[i][0];
			I_ref[2, 1] -=m * vertices[i][2] * vertices[i][1];
			I_ref[2, 2] -=m * vertices[i][2] * vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		Vector3 x = transform.position;
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
		int num = 0;
		Vector3 X = Vector3.zero;
		Vector3 V = Vector3.zero;
		Matrix4x4 I = R * I_ref * R.transpose; 
		// 碰撞检测
		for(int i = 0; i < vertices.Length; i++)
		{
			Vector3 xi = x + R.MultiplyPoint3x4(vertices[i]);
			Vector3 vi = v + Vector3.Cross(w, R.MultiplyPoint3x4(vertices[i]));
			if (CollideWithPlane(P, N, xi) && Vector3.Dot(vi, N) < 0)
			{
				num++;
				X += xi;
			}
		}

		if (num == 0)
			return;
		
		X /= num;
		Vector3 Rr = X - x;
		V = v + Vector3.Cross(w, Rr);
		
		// 碰撞响应
		Vector3 vn = Vector3.Dot(V, N) * N;
		Vector3 vt = V - vn;
		float a = Mathf.Max(1 - mu * (1 + restitution) * vn.magnitude / vt.magnitude, 0);
		vn = -restitution * vn;
		vt = a * vt;
		Vector3 Vnew = vn + vt;

		// 计算 K
		Matrix4x4 K = Matrix4x4.identity;
		K[0, 0] /= mass;
		K[1, 1] /= mass;
		K[2, 2] /= mass;
		K[3, 3] /= mass;
		Matrix4x4 tmp = Get_Cross_Matrix(Rr) * I.inverse * Get_Cross_Matrix(Rr);
		K[0, 0] -= tmp[0, 0];
		K[0, 1] -= tmp[0, 1];
		K[0, 2] -= tmp[0, 2];
		K[0, 3] -= tmp[0, 3];
		K[1, 0] -= tmp[1, 0];
		K[1, 1] -= tmp[1, 1];
		K[1, 2] -= tmp[1, 2];
		K[1, 3] -= tmp[1, 3];
		K[2, 0] -= tmp[2, 0];
		K[2, 1] -= tmp[2, 1];
		K[2, 2] -= tmp[2, 2];
		K[3, 3] -= tmp[3, 3];
		
		// 计算 j
		// Matrix4x4 deltaV = Matrix4x4.identity;
		// deltaV[0, 0] = Vnew.x - V.x;
		// deltaV[1, 0] = Vnew.y - V.y;
		// deltaV[2, 0] = Vnew.z - V.z;
		//K = K.inverse * deltaV;
		Vector3 j = K.inverse.MultiplyVector(Vnew - V);
		v += j / mass;
		// Vector3 tmp1 = Vector3.Cross(Rr, j);
		// tmp = Matrix4x4.identity;
		// tmp[0, 0] = tmp1.x;
		// tmp[1, 0] = tmp1.y;
		// tmp[2, 0] = tmp1.z;
		// tmp = I.inverse * tmp; 
		w += I.inverse.MultiplyVector(Vector3.Cross(Rr, j));

		restitution *= 0.9f;
		
		// 衰减碰撞因子
		if (v.magnitude < 0.01f || w.magnitude < 0.01f)
		{
			restitution = 0;
		}
	}

	bool CollideWithPlane(Vector3 P, Vector3 N, Vector3 x)
	{
		return Vector3.Dot((x - P), N) < 0;
	}

	// Update is called once per frame
	void FixedUpdate() 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched = false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			launched = true;
		}

		if (!launched)
			return;

		// Part I: Update velocities
		updateVelocity();
		updateAngular();

		// Part II: Collision Impulse
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

		// Part III: Update position & orientation
		//Update linear status
		Vector3 x    = transform.position;
		x = updatePosition(x);
		
		//Update angular status
		Quaternion q = transform.rotation;
		q = updateRotation(q);

		// Part IV: Assign to the object
		transform.position = x;
		transform.rotation = q;
	}

	// 更新速度
	void updateVelocity()
	{
		v += dt * gravity;
		v *= linear_decay;
	}

	// 根据速度更新位置
	Vector3 updatePosition(Vector3 x)
	{
		return x + v * dt;
	}
	
	// 更新角速度
	void updateAngular()
	{
		w *= angular_decay;
	}
	
	// 更新旋转角度
	Quaternion updateRotation(Quaternion q)
	{
		Vector3 tmp = w * dt / 2;
		Quaternion tmp1 = new Quaternion(tmp.x, tmp.y, tmp.z, 0) * q;
		return new Quaternion(tmp1.x + q.x, tmp1.y + q.y, tmp1.z + q.z, tmp1.w + q.w).normalized;
	}
}
