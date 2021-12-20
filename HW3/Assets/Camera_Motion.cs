using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Camera_Motion : MonoBehaviour
{
	bool pressed = false;

    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetMouseButtonDown (0)) 
		{
			pressed = true;
			Ray ray=Camera.main.ScreenPointToRay (Input.mousePosition);
		}
		if (Input.GetMouseButtonUp (0))
			pressed = false;

		if(pressed)
		{
			{
				float h;

				h = 5.0f * Input.GetAxis("Mouse Y");
				transform.Rotate(h, 0, 0);

				h = 5.0f * Input.GetAxis("Mouse X");
				Camera.main.transform.RotateAround(new Vector3(0, 0, 0), Vector3.up, h);
			}
		}
    }
}
