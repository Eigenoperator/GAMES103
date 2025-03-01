using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class FVM : MonoBehaviour
{
	float dt 			= 0.003f;
    float mass 			= 1;
	public float lamda	= 5000.0f;
    public float miu 	= 5000.0f;
	public float stiffness2 = 5000.0f;
	public float stiffness3 = 5000.0f;
    float damp			= 0.999f;
	Vector3 gravity 	= new Vector3(0, -9.8f, 0);
	int[] 		Tet;
	int tet_number;			//The number of tetrahedra
	float friction = 0.5f;
	float restitution = 0.5f;
	Vector3 P = new Vector3(0, -3.0f, 0);
	Vector3 N = new Vector3(0, 1.0f, 0);
	Vector3[] 	Force;
	Vector3[] 	V;
	Vector3[] 	X;
	int number;				//The number of vertices
	public float Laplacian_smoothing_rate = 0.9f;
    private LineRenderer lineRenderer;
	public bool IsDragging = false;

	Matrix4x4[] inv_Dm;


	//For Laplacian smoothing.
	Vector3[]   V_sum;
	int[]		V_num;

	SVD svd = new SVD();

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
	    	center=center/number;
	    	for(int i=0; i<number; i++)
	    	{
	    		X[i]-=center;
	    		float temp=X[i].y;
	    		X[i].y=X[i].z;
	    		X[i].z=temp;
	    	}

			
		}
        /*tet_number=1;
        Tet = new int[tet_number*4];
        Tet[0]=0;
        Tet[1]=1;
        Tet[2]=2;
        Tet[3]=3;

        number=4;
        X = new Vector3[number];
        V = new Vector3[number];
        Force = new Vector3[number];
        X[0]= new Vector3(0, 0, 0);
        X[1]= new Vector3(1, 0, 0);
        X[2]= new Vector3(0, 1, 0);
        X[3]= new Vector3(0, 0, 1);*/


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

			
		inv_Dm = new Matrix4x4[tet_number];
		for(int tet=0; tet<tet_number; tet++)
		{
			inv_Dm[tet] = Build_Edge_Matrix(tet).inverse;
		}
		
		// draw a line
        lineRenderer = gameObject.AddComponent<LineRenderer>();

        // properties of the line
        lineRenderer.positionCount = 2;  
        lineRenderer.startWidth = 0.05f; 
        lineRenderer.endWidth = 0.05f;   
        lineRenderer.material = new Material(Shader.Find("Sprites/Default")); 
        lineRenderer.startColor = Color.red; 
        lineRenderer.endColor = Color.red;   
    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
    	Matrix4x4 ret=Matrix4x4.zero;
    	//TODO: Need to build edge matrix here.
		Vector3 X10 = X[Tet[tet*4+1]]-X[Tet[tet*4+0]];
		Vector3 X20 = X[Tet[tet*4+2]]-X[Tet[tet*4+0]];
		Vector3 X30 = X[Tet[tet*4+3]]-X[Tet[tet*4+0]];
		ret.SetColumn(0, new Vector4(X10.x, X10.y, X10.z, 0));
		ret.SetColumn(1, new Vector4(X20.x, X20.y, X20.z, 0));
		ret.SetColumn(2, new Vector4(X30.x, X30.y, X30.z, 0));
		ret.SetColumn(3, new Vector4(0, 0, 0, 1));
		return ret;
	}

	Matrix4x4 Matrix4x4_add_Matrix4x4(Matrix4x4 a, Matrix4x4 b)
	{
		Matrix4x4 ret=Matrix4x4.zero;
		for(int i=0; i<4; i++)
			for(int j=0; j<4; j++)
				ret[i, j]=a[i, j]+b[i, j];
		return ret;
	}
	Matrix4x4 Matrix4x4_sub_Matrix4x4(Matrix4x4 a, Matrix4x4 b)
	{
		Matrix4x4 ret=Matrix4x4.zero;
		for(int i=0; i<4; i++)
			for(int j=0; j<4; j++)
				ret[i, j]=a[i, j]-b[i, j];
		return ret;
	}

	Matrix4x4 Matrix4x4_scalar_multi(float b, Matrix4x4 a)
	{
		Matrix4x4 ret=Matrix4x4.zero;
		for(int i=0; i<4; i++)
			for(int j=0; j<4; j++)
				ret[i, j]=a[i, j]*b;
		return ret;
	}

	Matrix4x4 Build_Dm(int tet)
	{
		Matrix4x4 ret=Matrix4x4.zero;


		return ret;
    }

	void Laplacian_Smoothing(float a)
	{
		for(int i = 0; i < number; i++)
		{
			V_sum[i] = Vector3.zero;
			V_num[i] = 0;
		}
		for(int tet = 0; tet < tet_number; tet++)
		{
			for(int i=0; i<4; i++)
			{
				V_sum[Tet[tet*4+i]] += X[Tet[tet*4+i]];
				V_num[Tet[tet*4+i]]++;
			}
		}
		for(int i = 0; i<number; i++)
		{
			if(V_num[i] > 0)
				X[i] = X[i] * a + V_sum[i] * (1.0f - a) / V_num[i];
		}
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
			Force[i] = Vector3.zero;
			Force[i] += gravity * mass;
    	}

    	for(int tet=0; tet<tet_number; tet++)
    	{
    		//TODO: Deformation Gradient
    		Matrix4x4 F = Build_Edge_Matrix(tet) * inv_Dm[tet];

    		// //TODO: Green Strain
			Matrix4x4 G = Matrix4x4_scalar_multi(0.5f, Matrix4x4_sub_Matrix4x4(F.transpose * F, Matrix4x4.identity));

    		// TODO: First PK Stress(FVM)
			float trace = G[0, 0] + G[1, 1] + G[2, 2];
			Matrix4x4 P = F * Matrix4x4_add_Matrix4x4(Matrix4x4_scalar_multi(2.0f * miu, G), Matrix4x4_scalar_multi(lamda * trace, Matrix4x4.identity));


			// // TODO: First PK Stress(FEM)
			// Matrix4x4 U = Matrix4x4.zero;
			// Matrix4x4 S = Matrix4x4.zero;
			// Matrix4x4 V = Matrix4x4.zero;
			// float lamda0, lamda1, lamda2;
			// svd.svd(F, ref U, ref S, ref V);
			// lamda0 = S[0, 0];
			// lamda1 = S[1, 1];
			// lamda2 = S[2, 2];
			// float I1 = lamda0 * lamda0 + lamda1 * lamda1 + lamda2 * lamda2;
			// float I2 = lamda0 * lamda0 * lamda0 * lamda0 + lamda1 * lamda1 * lamda1 * lamda1 + lamda2 * lamda2 * lamda2 * lamda2;
			// float I3 = lamda0 * lamda0 * lamda1 * lamda1 * lamda2 * lamda2;
			// Matrix4x4 P0 = Matrix4x4.zero;

			// // StVK model
			// P0[0, 0] = lamda0 * (2.0f * lamda * (I1 - 3.0f) + miu * (lamda0 * lamda0 - 1.0f));
			// P0[1, 1] = lamda1 * (2.0f * lamda * (I1 - 3.0f) + miu * (lamda1 * lamda1 - 1.0f));
			// P0[2, 2] = lamda2 * (2.0f * lamda * (I1 - 3.0f) + miu * (lamda2 * lamda2 - 1.0f));
			// P0[3,3] = 1.0f;
			// Matrix4x4 P = U * P0 * V.transpose;

			//neo-Hookean
			// Matrix4x4 P = Matrix4x4_add_Matrix4x4(Matrix4x4_scalar_multi(miu, Matrix4x4_sub_Matrix4x4(F, F.inverse.transpose)), Matrix4x4_scalar_multi(lamda * (float)Math.Log(I3) / 2.0f, F.inverse.transpose));

			//Mooney-Rivlin
			// P0[0, 0] = lamda * (- 4.0f / 3.0f * (float)Math.Pow(I3, -4.0f / 3.0f) * (float)Math.Pow(lamda0, 3) * I1 + (float)Math.Pow(I3, -1.0f / 3.0f) * 2.0f * lamda0) 
			// 		 - 2.0f * miu * (float)Math.Pow(I3, -3.0f / 2.0f) * (float)Math.Pow(lamda0, 3)
			// 		 + stiffness2 / 2.0f * (- 2.0f / 3.0f * (float)Math.Pow(I3, -5.0f / 3.0f) * (I1 * I1 - I2) + 2.0f * (float)Math.Pow(I3, -2.0f / 3.0f)) * (2.0f * I1 * lamda0 - lamda0 * lamda1 * lamda1 - lamda0 * lamda2 * lamda2);
			// P0[1, 1] = lamda * (- 4.0f / 3.0f * (float)Math.Pow(I3, -4.0f / 3.0f) * (float)Math.Pow(lamda1, 3) * I1 + (float)Math.Pow(I3, -1.0f / 3.0f) * 2.0f * lamda1)
			// 		 - 2.0f * miu * (float)Math.Pow(I3, -3.0f / 2.0f) * (float)Math.Pow(lamda1, 3)
			// 		 + stiffness2 / 2.0f * (- 2.0f / 3.0f * (float)Math.Pow(I3, -5.0f / 3.0f) * (I1 * I1 - I2) + 2.0f * (float)Math.Pow(I3, -2.0f / 3.0f)) * (2.0f * I1 * lamda1 - lamda1 * lamda0 * lamda0 - lamda1 * lamda2 * lamda2);
			// P0[2, 2] = lamda * (- 4.0f / 3.0f * (float)Math.Pow(I3, -4.0f / 3.0f) * (float)Math.Pow(lamda2, 3) * I1 + (float)Math.Pow(I3, -1.0f / 3.0f) * 2.0f * lamda2)
			// 		 - 2.0f * miu * (float)Math.Pow(I3, -3.0f / 2.0f) * (float)Math.Pow(lamda2, 3)
			// 		 + stiffness2 / 2.0f * (- 2.0f / 3.0f * (float)Math.Pow(I3, -5.0f / 3.0f) * (I1 * I1 - I2) + 2.0f * (float)Math.Pow(I3, -2.0f / 3.0f)) * (2.0f * I1 * lamda2 - lamda2 * lamda0 * lamda0 - lamda2 * lamda1 * lamda1);
			// P0[3, 3] = 1.0f;
			// Matrix4x4 P = U * P0 * V.transpose;





			



    		//TODO: Elastic Force
			Matrix4x4 elastic_force = Matrix4x4_scalar_multi(- 1.0f / (6.0f * inv_Dm[tet].determinant), (P * inv_Dm[tet].transpose));
			Vector3 f1 = new Vector3(elastic_force[0, 0], elastic_force[1, 0], elastic_force[2, 0]);
			Vector3 f2 = new Vector3(elastic_force[0, 1], elastic_force[1, 1], elastic_force[2, 1]);
			Vector3 f3 = new Vector3(elastic_force[0, 2], elastic_force[1, 2], elastic_force[2, 2]);
			Vector3 f0 = -f1 - f2 - f3;

			//TODO: Add elastic force to Force
			Force[Tet[tet*4+0]] += f0;
			Force[Tet[tet*4+1]] += f1;
			Force[Tet[tet*4+2]] += f2;
			Force[Tet[tet*4+3]] += f3;
    	}
		Laplacian_Smoothing(Laplacian_smoothing_rate);


		int index = -1;
		//dragging
		if (Input.GetMouseButton(0)) 
		{
        
        	Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        	RaycastHit hit;
        
			
        	if (Physics.Raycast(ray, out hit))
        	{
            	Vector3 hitPoint = hit.point;
            	Transform hitTransform = hit.transform; 

				MeshFilter meshFilter = hitTransform.GetComponent<MeshFilter>();
				// Debug.Log(meshFilter);
            	if (hit.collider.gameObject == gameObject)
            	{	
					lineRenderer.enabled = true;
					Debug.Log("Hit the object");
               	 	Mesh mesh = meshFilter.mesh;
                	Vector3[] vertices = mesh.vertices; 
	
					if (index == -1 && IsDragging == false)
					{
                		for (int i = 0; i < X.Length; i++)
                		{             
                    		Vector3 worldVertex = hitTransform.TransformPoint(X[i]);
                    		float distance = Vector3.Distance(hitPoint, worldVertex);
                    		if (distance < 0.5f) 
                    		{	
								//change the force here later
								IsDragging = true;
								Vector3 closestVertex = hit.collider.ClosestPointOnBounds(hit.point);
                      	  		Debug.Log("Hit point is closest to vertex at index: " + i);
								index = i;
								break;
                    		}
                		}
					}
					if (Input.GetMouseButton(0) && index != -1 && IsDragging)
					{
						Vector3 mousePosition = Input.mousePosition; 
						mousePosition.z = 10.0f;
						Vector3 mouseworldPosition = Camera.main.ScreenToWorldPoint(mousePosition);
						Vector3 selectedVertex_wotld = hitTransform.TransformPoint(X[index]);
						Force[index] += (mouseworldPosition - selectedVertex_wotld) * 700.0f;
						float distance = Vector3.Distance(mouseworldPosition, selectedVertex_wotld);
						

						// Debug.Log("Mouse world position: " + mouseworldPosition);
						
						lineRenderer.SetPosition(0, X[index]); // start
        				lineRenderer.SetPosition(1, mouseworldPosition); // end(mouse position)
					}				
            	}
        	}
   	 	}

		

		if (Input.GetMouseButtonUp(0))
		{	
			index = -1;		
			lineRenderer.enabled = false; //disabe the line renderer
			IsDragging = false;
		}
		
		if (Input.GetKeyDown(KeyCode.R))
		{
			Force[0] = new Vector3(2000.0f, 0, 0);
		}

		// if(Input.GetMouseButton(0))
		// {
		// 	Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
		// 	RaycastHit hit;
		// 	if(Physics.Raycast(ray, out hit))
		// 	{
		// 		Vector3 hitPoint = hit.point;
		// 		for(int i=0; i<number; i++)
		// 		{
					
		// 		}
		// 	}
		// }

    	for(int i=0; i<number; i++)
    	{
    		//TODO: Update X and V here.
			V[i] = (V[i] + Force[i] * dt / mass) * damp;
			X[i] += V[i] * dt;

    		//TODO: (Particle) collision with floor.
			float d = Vector3.Dot(X[i] - P, N);
            if (d < 0.0f) // collision occur
            {
                float v_N_size = Vector3.Dot(V[i], N);
                
                if (v_N_size < 0.0f)
                {
                    Vector3 v_N = v_N_size * N;
                    Vector3 v_T = V[i] - v_N;
                    Vector3 v_N_new = -1.0f * restitution * v_N;
                    float a = Math.Max(1.0f - friction * (1.0f + restitution) * v_N.magnitude / v_T.magnitude, 0.0f);
                    Vector3 v_T_new = a * v_T;
                    V[i] = v_N_new + v_T_new;
                }
            }
			
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
}
