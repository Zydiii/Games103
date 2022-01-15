using UnityEngine;
using System.Collections;

public class wave_motion : MonoBehaviour 
{
	int size 		= 100;
	float rate 		= 0.005f;
	float gamma		= 0.004f;
	float damping 	= 0.996f;
	float[,] 	old_h;
	float[,]	low_h;
	float[,]	vh;
	float[,]	b;

	bool [,]	cg_mask;
	float[,]	cg_p;
	float[,]	cg_r;
	float[,]	cg_Ap;
	bool 	tag=true;

	Vector3 	cube_v = Vector3.zero;
	Vector3 	cube_w = Vector3.zero;
	private float cube_mass;
	private Vector3 cube_force = Vector3.zero;
	private Vector3 gravity = new Vector3(0, -9.8f, 0);
	Vector3 cube_torque = Vector3.zero;
	private float rhoWater = 1.0f;

	private GameObject cube;
	private Bounds cube_bounds;
	private GameObject block;
	private Bounds block_bounds;

	private float dt;
	
	Matrix4x4 I_ref;

	// Use this for initialization
	void Start () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.Clear ();

		Vector3[] X=new Vector3[size*size];

		// 每格 0.1f, 以原点为中心
		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			X[i*size+j].x=i*0.1f-size*0.05f;
			X[i*size+j].y=0;
			X[i*size+j].z=j*0.1f-size*0.05f;
		}

		int[] T = new int[(size - 1) * (size - 1) * 6];
		int index = 0;
		for (int i=0; i<size-1; i++)
		for (int j=0; j<size-1; j++)
		{
			T[index*6+0]=(i+0)*size+(j+0);
			T[index*6+1]=(i+0)*size+(j+1);
			T[index*6+2]=(i+1)*size+(j+1);
			T[index*6+3]=(i+0)*size+(j+0);
			T[index*6+4]=(i+1)*size+(j+1);
			T[index*6+5]=(i+1)*size+(j+0);
			index++;
		}
		mesh.vertices  = X;
		mesh.triangles = T;
		mesh.RecalculateNormals ();

		low_h 	= new float[size,size];
		old_h 	= new float[size,size];
		vh 	  	= new float[size,size];
		b 	  	= new float[size,size];

		cg_mask	= new bool [size,size];
		cg_p 	= new float[size,size];
		cg_r 	= new float[size,size];
		cg_Ap 	= new float[size,size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			low_h[i,j]=99999;
			old_h[i,j]=0;
			vh[i,j]=0;
		}
		
		// Get object
		cube = GameObject.Find("Cube");
		cube_bounds = cube.GetComponent<MeshFilter>().mesh.bounds;
		block = GameObject.Find("Block");
		block_bounds = block.GetComponent<MeshFilter>().mesh.bounds;
		
		dt = Time.deltaTime / 8;

		Vector3[] cube_vertices = cube.GetComponent<MeshFilter>().mesh.vertices;
		// Get I
		for (int i = 0; i < cube_vertices.Length; i++) 
		{
			cube_mass += 1;
			float diag = cube_vertices[i].sqrMagnitude;
			I_ref[0, 0] += diag;
			I_ref[1, 1] += diag;
			I_ref[2, 2] += diag;
			I_ref[0, 0] -= cube_vertices[i][0] * cube_vertices[i][0];
			I_ref[0, 1] -= cube_vertices[i][0] * cube_vertices[i][1];
			I_ref[0, 2] -= cube_vertices[i][0] * cube_vertices[i][2];
			I_ref[1, 0] -= cube_vertices[i][1] * cube_vertices[i][0];
			I_ref[1, 1] -= cube_vertices[i][1] * cube_vertices[i][1];
			I_ref[1, 2] -= cube_vertices[i][1] * cube_vertices[i][2];
			I_ref[2, 0] -= cube_vertices[i][2] * cube_vertices[i][0];
			I_ref[2, 1] -= cube_vertices[i][2] * cube_vertices[i][1];
			I_ref[2, 2] -= cube_vertices[i][2] * cube_vertices[i][2];
		}
		I_ref[3, 3] = 1;
	}

	void A_Times(bool[,] mask, float[,] x, float[,] Ax, int li, int ui, int lj, int uj)
	{
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			Ax[i,j]=0;
			if(i!=0)		Ax[i,j]-=x[i-1,j]-x[i,j];
			if(i!=size-1)	Ax[i,j]-=x[i+1,j]-x[i,j];
			if(j!=0)		Ax[i,j]-=x[i,j-1]-x[i,j];
			if(j!=size-1)	Ax[i,j]-=x[i,j+1]-x[i,j];
		}
	}

	float Dot(bool[,] mask, float[,] x, float[,] y, int li, int ui, int lj, int uj)
	{
		float ret=0;
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			ret+=x[i,j]*y[i,j];
		}
		return ret;
	}

	void Conjugate_Gradient(bool[,] mask, float[,] b, float[,] x, int li, int ui, int lj, int uj)
	{
		//Solve the Laplacian problem by CG.
		A_Times(mask, x, cg_r, li, ui, lj, uj);

		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			cg_p[i,j]=cg_r[i,j]=b[i,j]-cg_r[i,j];
		}

		float rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);

		for(int k=0; k<128; k++)
		{
			if(rk_norm<1e-10f)	break;
			A_Times(mask, cg_p, cg_Ap, li, ui, lj, uj);
			float alpha=rk_norm/Dot(mask, cg_p, cg_Ap, li, ui, lj, uj);

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				x[i,j]   +=alpha*cg_p[i,j];
				cg_r[i,j]-=alpha*cg_Ap[i,j];
			}

			float _rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);
			float beta=_rk_norm/rk_norm;
			rk_norm=_rk_norm;

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				cg_p[i,j]=cg_r[i,j]+beta*cg_p[i,j];
			}
		}

	}

	void Shallow_Wave(float[,] old_h, float[,] h, float [,] new_h)
	{	
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X    = mesh.vertices;
		//Step 1:
		//TODO: Compute new_h based on the shallow wave model.
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				new_h[i, j] = h[i, j] + damping * (h[i, j] - old_h[i, j]);
				if (i != 0)
					new_h[i, j] += rate * (h[i - 1, j] - h[i, j]);
				if(i != size - 1)
					new_h[i, j] += rate * (h[i + 1, j] - h[i, j]);
				if(j != 0)
					new_h[i, j] += rate * (h[i, j - 1] - h[i, j]);
				if(j != size - 1)
					new_h[i, j] += rate * (h[i, j + 1] - h[i, j]);
			}
		}
		
		//Step 2: Object->Water Coupling
		//TODO: for block 1, calculate low_h.
		Vector3 block_pos = block.transform.position;
		int block_li = (int) ((block_pos.x + 5.0f) / 0.1) - 4;
		int block_ui = (int) ((block_pos.x + 5.0f) / 0.1) + 4;
		int block_lj = (int) ((block_pos.z + 5.0f) / 0.1) - 4;
		int block_uj = (int) ((block_pos.z + 5.0f) / 0.1) + 4;
		for (int i = block_li; i <= block_ui; i++)
		{
			for (int j = block_lj; j <= block_uj; j++)
			{
				if (i >= 0 && j >= 0 && i < size && j < size)
				{
					Vector3 ray_pos_0 = block.transform.InverseTransformPoint(new Vector3(X[i * size + j].x, -10, X[i * size + j].z));
					Vector3 ray_pos_1 = block.transform.InverseTransformPoint(new Vector3(X[i * size + j].x, 0, X[i * size + j].z));
					Ray ray = new Ray(ray_pos_0, ray_pos_1 - ray_pos_0);
					float dist = 99999;
					block_bounds.IntersectRay(ray, out dist);
					low_h[i, j] = -10 + dist;
				}
			}
		}
		//TODO: then set up b and cg_mask for conjugate gradient.
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				if (low_h[i, j] > h[i, j])
				{
					b[i, j] = 0;
					cg_mask[i, j] = false;
					vh[i, j] = 0;
				}
				else
				{
					b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
					cg_mask[i, j] = true;
				}
			}
		}
		//TODO: Solve the Poisson equation to obtain vh (virtual height).
		Conjugate_Gradient(cg_mask, b, vh, block_li, block_ui, block_lj, block_uj);
		
		//TODO: for block 2, calculate low_h.
		Vector3 cube_pos = cube.transform.position;
		int cube_li = (int) ((cube_pos.x + 5.0f) / 0.1) - 4;
		int cube_ui = (int) ((cube_pos.x + 5.0f) / 0.1) + 4;
		int cube_lj = (int) ((cube_pos.z + 5.0f) / 0.1) - 4;
		int cube_uj = (int) ((cube_pos.z + 5.0f) / 0.1) + 4;
		for (int i = cube_li; i <= cube_ui; i++)
		{
			for (int j = cube_lj; j <= cube_uj; j++)
			{
				if (i >= 0 && j >= 0 && i < size && j < size)
				{
					Vector3 ray_pos_0 = cube.transform.InverseTransformPoint(new Vector3(X[i * size + j].x, -10, X[i * size + j].z));
					Vector3 ray_pos_1 = cube.transform.InverseTransformPoint(new Vector3(X[i * size + j].x, 0, X[i * size + j].z));
					Ray ray = new Ray(ray_pos_0, ray_pos_1 - ray_pos_0);
					float dist = 99999;
					cube_bounds.IntersectRay(ray, out dist);
					low_h[i, j] = -10 + dist;
				}
			}
		}
		//TODO: then set up b and cg_mask for conjugate gradient.
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				if (low_h[i, j] > h[i, j])
				{
					b[i, j] = 0;
					cg_mask[i, j] = false;
					vh[i, j] = 0;
				}
				else
				{
					b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
					cg_mask[i, j] = true;
				}
			}
		}
		//TODO: Solve the Poisson equation to obtain vh (virtual height).
		Conjugate_Gradient(cg_mask, b, vh, cube_li, cube_ui, cube_lj, cube_uj);
		
		//TODO: Diminish vh.
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++) 
			{
				if(cg_mask[i,j])
					vh[i,j] *= gamma;
			}
		}
		
		//TODO: Update new_h by vh.
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				if (i != 0)
					new_h[i, j] += rate * (vh[i - 1, j] - vh[i, j]);
				if(i != size - 1)
					new_h[i, j] += rate * (vh[i + 1, j] - vh[i, j]);
				if(j != 0)
					new_h[i, j] += rate * (vh[i, j - 1] - vh[i, j]);
				if(j != size - 1)
					new_h[i, j] += rate * (vh[i, j + 1] - vh[i, j]);
			}
		}
		
		//Step 3
		//TODO: old_h <- h; h <- new_h;
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				old_h[i, j] = h[i, j];
				h[i, j] = new_h[i, j];
			}
		}

		//Step 4: Water->Block coupling.
		cube_force = cube_mass * gravity;
		cube_torque = new Vector3(0, 0, 0);
        // get force and torque
		for (int i = cube_li; i <= cube_ui; i++)
		{
			for (int j = cube_lj; j <= cube_uj; j++)
				if (i >= 0 && j >= 0 && i < size && j < size)
				{
					Vector3 ray_pos_0 = cube.transform.InverseTransformPoint(new Vector3(X[i * size + j].x, -10, X[i * size + j].z));
					Vector3 ray_pos_1 = cube.transform.InverseTransformPoint(new Vector3(X[i * size + j].x, 0, X[i * size + j].z));
					Ray ray = new Ray(ray_pos_0, ray_pos_1 - ray_pos_0);
					float dist = 99999;
					cube_bounds.IntersectRay(ray, out dist);
					if (vh[i, j] != 0)
					{
						Vector3 r = ray_pos_0 + dist * (ray_pos_1 - ray_pos_0).normalized - cube_pos;
						Vector3 f = new Vector3(0, vh[i, j], 0) * rhoWater * Mathf.Abs(gravity.y);
						cube_force += f;
						cube_torque += Vector3.Cross(r, f);
					}
				}
		}
		
		// velocity
		Quaternion q = cube.transform.rotation;
		Matrix4x4 R = Matrix4x4.Rotate(q);
		Matrix4x4 I = R * I_ref * R.transpose;
		cube_v += cube_force * dt / cube_mass;
		cube_v *= damping;
		//cube_w += I.inverse.MultiplyVector(cube_torque) * dt; // 这个方法不稳定
		cube_w += cube_torque * dt / cube_mass / 100;
		cube_w *= damping;

		// pos
		cube.transform.position += cube_v * dt;

		// rotation
		Quaternion temp = new Quaternion(cube_w.x * dt / 2, cube_w.y * dt / 2, cube_w.z * dt / 2, 0) * q;
		q.x += temp.x;
		q.y += temp.y;
		q.z += temp.z;
		q.w += temp.w;
		cube.transform.rotation = q;
	}
	

	// Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X    = mesh.vertices;
		float[,] new_h = new float[size, size];
		float[,] h     = new float[size, size];

		//TODO: Load X.y into h.
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				h[i, j] = X[i * size + j].y;
			}
		}

		if (Input.GetKeyDown ("r")) 
		{
			//TODO: Add random water.
			int i = Random.Range(1, size - 2);
			int j = Random.Range(1, size - 2);
			float r = Random.Range(0.1f, 0.3f);
			h[i, j] += r;
			h[i - 1, j - 1] -= r / 4.0f;
			h[i + 1, j - 1] -= r / 4.0f;
			h[i - 1, j + 1] -= r / 4.0f;
			h[i + 1, j + 1] -= r / 4.0f;
		}
	
		for(int l = 0; l < 8; l++)
		{
			Shallow_Wave(old_h, h, new_h);
		}

		//TODO: Store h back into X.y and recalculate normal.
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				X[i * size + j].y = h[i, j];
			}
		}
		
		mesh.vertices = X;
		mesh.RecalculateNormals();

	}
}
