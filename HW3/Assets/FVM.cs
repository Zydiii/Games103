using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class FVM : MonoBehaviour
{
	float dt 			= 0.003f;
    float mass 			= 1;
	float stiffness_0	= 20000.0f;
    float stiffness_1 	= 5000.0f;
    float damp			= 0.999f;

	int[] 		Tet;
	int tet_number;			//The number of tetrahedra

	Vector3[] 	Force;
	Vector3[] 	V;
	Vector3[] 	X;
	int number;				//The number of vertices

	Matrix4x4[] inv_Dm;

	//For Laplacian smoothing.
	Vector3[]   V_sum;
	int[]		V_num;

	private Vector3 gravity = new Vector3(0, -9.8f, 0);
	private Vector3 floorPos = new Vector3(0, -3, 0);
	private Vector3 floorNormal = new Vector3(0, 1, 0);
	private float muN = 0.5f;
	private float muT = 0.5f;
	private float blendAlpha = 0.5f;

	SVD svd = new SVD();
	[Header("附加题")]
	public bool useBonus = false;

    // Start is called before the first frame update
    void Start()
    {
    	// FILO IO: Read the house model from files.
    	// The model is from Jonathan Schewchuk's Stellar lib.
    	{
    		string fileContent = File.ReadAllText("Assets/house2.ele");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		
    		tet_number=int.Parse(Strings[0]);
        	Tet = new int[tet_number*4];

    		for(int tet=0; tet<tet_number; tet++)
    		{
				Tet[tet*4+0]=int.Parse(Strings[tet*5+4])-1;
				Tet[tet*4+1]=int.Parse(Strings[tet*5+5])-1;
				Tet[tet*4+2]=int.Parse(Strings[tet*5+6])-1;
				Tet[tet*4+3]=int.Parse(Strings[tet*5+7])-1;
			}
    	}
    	{
			string fileContent = File.ReadAllText("Assets/house2.node");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		number = int.Parse(Strings[0]);
    		X = new Vector3[number];
       		for(int i=0; i<number; i++)
       		{
       			X[i].x=float.Parse(Strings[i*5+5])*0.4f;
       			X[i].y=float.Parse(Strings[i*5+6])*0.4f;
       			X[i].z=float.Parse(Strings[i*5+7])*0.4f;
       		}
    		//Centralize the model.
	    	Vector3 center=Vector3.zero;
	    	for(int i=0; i<number; i++)		center+=X[i];
	    	center /= number;
	    	for(int i=0; i<number; i++)
	    	{
	    		X[i]-=center;
	    		float temp=X[i].y;
	    		X[i].y=X[i].z;
	    		X[i].z=temp;
	    	}
		}
        
        //Create triangle mesh.
       	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];

        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }

        int[] triangles = new int[tet_number*12];
        for(int t=0; t<tet_number*4; t++)
        {
        	triangles[t*3+0]=t*3+0;
        	triangles[t*3+1]=t*3+1;
        	triangles[t*3+2]=t*3+2;
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals ();

		V 	  = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];

		//TODO: Need to allocate and assign inv_Dm
		inv_Dm = new Matrix4x4[tet_number];
		for (int i = 0; i < tet_number; i++)
		{
			inv_Dm[i] = Build_Edge_Matrix(i).inverse;
		}
    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
    	Matrix4x4 ret=Matrix4x4.zero;
    	//TODO: Need to build edge matrix here.
        ret[0, 0] = X[Tet[tet * 4 + 1]].x - X[Tet[tet * 4]].x;
        ret[1, 0] = X[Tet[tet * 4 + 1]].y - X[Tet[tet * 4]].y;
        ret[2, 0] = X[Tet[tet * 4 + 1]].z - X[Tet[tet * 4]].z;
        
        ret[0, 1] = X[Tet[tet * 4 + 2]].x - X[Tet[tet * 4]].x;
        ret[1, 1] = X[Tet[tet * 4 + 2]].y - X[Tet[tet * 4]].y;
        ret[2, 1] = X[Tet[tet * 4 + 2]].z - X[Tet[tet * 4]].z;
        
        ret[0, 2] = X[Tet[tet * 4 + 3]].x - X[Tet[tet * 4]].x;
        ret[1, 2] = X[Tet[tet * 4 + 3]].y - X[Tet[tet * 4]].y;
        ret[2, 2] = X[Tet[tet * 4 + 3]].z - X[Tet[tet * 4]].z;

        ret[3, 3] = 1;
        
        return ret;
    }

    void _Update()
    {
    	// Jump up.
		if(Input.GetKeyDown(KeyCode.Space))
    	{
    		for(int i=0; i<number; i++)
    			V[i].y+=0.2f;
    	}

    	for(int i=0 ;i<number; i++)
    	{
    		//TODO: Add gravity to Force.
            Force[i] = gravity * mass;
        }

    	for(int tet=0; tet<tet_number; tet++)
        {
	        //TODO: Deformation Gradient
	        Matrix4x4 F = Build_Edge_Matrix(tet) * inv_Dm[tet];
	        Matrix4x4 force;
	        if (!useBonus)
	        {
		        //TODO: Green Strain
		        Matrix4x4 G = matrixMultiplyFloat(matrixSubmatrix( F.transpose * F, Matrix4x4.identity), 0.5f);
		        //TODO: Second PK Stress
		        Matrix4x4 S = matrixAddmatrix(matrixMultiplyFloat(G, 2 * stiffness_1),
			        matrixMultiplyFloat(Matrix4x4.identity, stiffness_0 * trace(G)));
		        //TODO: Elastic Force
		        force = matrixMultiplyFloat(F * S * inv_Dm[tet].transpose, -1 / (inv_Dm[tet].determinant * 6));
	        }
	        else
	        {
		        Matrix4x4 U = Matrix4x4.zero;
		        Matrix4x4 A = Matrix4x4.zero;
		        Matrix4x4 V = Matrix4x4.zero;
		        Matrix4x4 P = Matrix4x4.zero;
		        svd.svd(F, ref U, ref A, ref V);
		        float Ic = trace2(A);
		        float IIc = trace4(A);
		        float IIIc = det2(A);
		        float dWdIc = 0.25f * stiffness_0 * (Ic - 3) - 0.5f * stiffness_1;
		        float dWdIIc = 0.25f * stiffness_1;
		        float dWdIIIc = 0;
		        float dIcdlamda0 = 2 * A[0, 0];
		        float dIcdlamda1 = 2 * A[1, 1];
		        float dIcdlamda2 = 2 * A[2, 2];
		        float dIIcdlamda0 = 4 * A[0, 0] * A[0, 0] * A[0, 0] ;
		        float dIIcdlamda1 = 4 * A[1, 1] * A[1, 1] * A[1, 1] ;
		        float dIIcdlamda2 = 4 * A[2, 2] * A[2, 2] * A[2, 2];
		        float dIIIcdlamda0 = 2 * IIIc / A[0, 0];
		        float dIIIcdlamda1 = 2 * IIIc / A[1, 1];
		        float dIIIcdlamda2 = 2 * IIIc / A[2, 2];
		        P[0, 0] = dWdIc * dIcdlamda0 + dWdIIc * dIIcdlamda0 + dWdIIIc * dIIIcdlamda0;
		        P[1, 1] = dWdIc * dIcdlamda1 + dWdIIc * dIIcdlamda1 + dWdIIIc * dIIIcdlamda1;
		        P[2, 2] = dWdIc * dIcdlamda2 + dWdIIc * dIIcdlamda2 + dWdIIIc * dIIIcdlamda2;
		        //TODO: Elastic Force
		        force = matrixMultiplyFloat(U * P * V.transpose * inv_Dm[tet].transpose, -1 / (inv_Dm[tet].determinant * 6));
	        }
	        Force[Tet[tet*4]].x += -1 * (force[0,0] + force[0,1] + force[0,2]);
	        Force[Tet[tet*4]].y += -1 * (force[1,0] + force[1,1] + force[1,2]);
	        Force[Tet[tet*4]].z += -1 * (force[2,0] + force[2,1] + force[2,2]);
	        Force[Tet[tet*4+1]].x += force[0,0];
	        Force[Tet[tet*4+1]].y += force[1,0];
	        Force[Tet[tet*4+1]].z += force[2,0];
	        Force[Tet[tet*4+2]].x += force[0,1];
	        Force[Tet[tet*4+2]].y += force[1,1];
	        Force[Tet[tet*4+2]].z += force[2,1];
	        Force[Tet[tet*4+3]].x += force[0,2];
	        Force[Tet[tet*4+3]].y += force[1,2];
	        Force[Tet[tet*4+3]].z += force[2,2];
        }
        
    	for(int i = 0; i < number; i++)
    	{
    		//TODO: Update X and V here.
            V[i] += dt * Force[i] / mass;
            V[i] *= damp;
            X[i] += dt * V[i];
            
            //TODO: (Particle) collision with floor.
            float signedDis = Vector3.Dot(X[i] - floorPos, floorNormal);
            if (signedDis < 0 && Vector3.Dot(V[i], floorNormal) < 0)
            {
	            X[i] -= signedDis * floorNormal;
	            Vector3 vN = Vector3.Dot(V[i], floorNormal) * floorNormal;
	            Vector3 vT = V[i] - vN;
	            float a = Math.Max(1 - muT * (1 + muN) * vN.magnitude / vT.magnitude, 0);
	            V[i] = -muN * vN + a * vT;
            }
        }
        
        Laplacian_Smoothing();
    }
    
    void Laplacian_Smoothing()
    {
	    for(int i = 0; i < number; i++)
	    {
		    V_sum[i] = new Vector3(0, 0, 0);
		    V_num[i] = 0;
	    }

	    for(int tet = 0; tet < tet_number; tet++)
	    {
		    Vector3 sum = V[Tet[tet*4]] + V[Tet[tet*4+1]] + V[Tet[tet*4+2]] + V[Tet[tet*4+3]];
		    V_sum[Tet[tet*4]] += sum - V[Tet[tet*4]];
		    V_sum[Tet[tet*4+1]] += sum - V[Tet[tet*4+1]];
		    V_sum[Tet[tet*4+2]] += sum - V[Tet[tet*4+2]];
		    V_sum[Tet[tet*4+3]] += sum - V[Tet[tet*4+3]];
		    V_num[Tet[tet*4]] += 3;
		    V_num[Tet[tet*4+1]] += 3;
		    V_num[Tet[tet*4+2]] += 3;
		    V_num[Tet[tet*4+3]] += 3;
	    }

	    for(int i = 0; i < number; i++)
	    {
		    V[i] = blendAlpha * V[i] + (1 - blendAlpha) * V_sum[i] / V_num[i];
	    }
    }

    // Update is called once per frame
    void Update()
    {
    	for(int l=0; l<10; l++)
    		 _Update();

    	// Dump the vertex array for rendering.
    	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.RecalculateNormals ();
    }

    Matrix4x4 matrixSubmatrix(Matrix4x4 a, Matrix4x4 b)
    {
	    for (int i = 0; i < 3; i++)
	    {
		    for (int j = 0; j < 3; j++)
		    {
			    a[i, j] -= b[i, j];
		    }
	    }

	    return a;
    }
    
    Matrix4x4 matrixAddmatrix(Matrix4x4 a, Matrix4x4 b)
    {
	    for (int i = 0; i < 3; i++)
	    {
		    for (int j = 0; j < 3; j++)
		    {
			    a[i, j] += b[i, j];
		    }
	    }

	    return a;
    }

    Matrix4x4 matrixMultiplyFloat(Matrix4x4 a, float num)
    {
	    for (int i = 0; i < 3; i++)
	    {
		    for (int j = 0; j < 3; j++)
		    {
			    a[i, j] *= num;
		    }
	    }

	    return a;
    }

    float trace(Matrix4x4 a)
    {
	    return a[0, 0] + a[1, 1] + a[2, 2];
    }

    float trace2(Matrix4x4 a)
    {
	    return (float) (Math.Pow(a[0, 0], 2) + Math.Pow(a[1, 1], 2) + Math.Pow(a[2, 2], 2));
    }
    
    float trace4(Matrix4x4 a)
    {
	    return (float) (Math.Pow(a[0, 0], 4) + Math.Pow(a[1, 1], 4) + Math.Pow(a[2, 2], 4));
    }

    float det2(Matrix4x4 a)
    {
	    return (float) (Math.Pow(a[0, 0], 2) * Math.Pow(a[1, 1], 2) * Math.Pow(a[2, 2], 2));
    }
}
