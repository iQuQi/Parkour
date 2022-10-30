using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityStandardAssets.CrossPlatformInput;

[RequireComponent(typeof(PlayerController))]
public class PlayerInputController : MonoBehaviour
{
    public Text inputText; // 디버깅 관련인듯

    private PlayerController m_Character;     // 깃헙에 정의된 클래스
    private Transform m_Cam;                  // 씬(scene)에서 기본 카메라에 대한 참조 transform
    private Vector3 m_CamForward;             // 카메라의 현재 forward 방향
    private Vector3 m_Move;
    private bool m_Jump;                      // camForward 및 사용자 입력으로부터 계산된 world-relative 이동 방향


    private void Start()
    {
        // get the transform of the main camera
        if (Camera.main != null)
        {
            m_Cam = Camera.main.transform;
        }
        else
        {
            Debug.LogWarning(
                "Warning: no main camera found. Third person character needs a Camera tagged \"MainCamera\", for camera-relative controls.", gameObject);
            // 이 경우에 자가 인식 컨트롤을 사용. 사용자가 원하는 것이 아닐거라서 경고를 해주긴함.
        }

        m_Character = GetComponent<PlayerController>(); // 게임 오브젝트의 컴포넌트를 가져오는 함수 - 구성 요소가 필요하므로 null일 수 없다
    }


    private void Update()
    {
        if (!m_Jump)
        {
            m_Jump = CrossPlatformInputManager.GetButtonDown("Jump"); // 사용자가 '점프'를 입력한 경우 m_Jump = true
        }
    }


    // Fixed update is called in sync with physics (물리엔진?..)
    private void FixedUpdate()
    {
        // 입력값 받기
        float h = CrossPlatformInputManager.GetAxis("Horizontal");
        float v = CrossPlatformInputManager.GetAxis("Vertical");
        bool crouch = Input.GetKey(KeyCode.C);

        // 캐릭터 전달을 위한 이동 방향을 계산
        if (m_Cam != null)
        {
            // 이동할 카메라 상대 방향 계산
            m_CamForward = Vector3.Scale(m_Cam.forward, new Vector3(1, 0, 1)).normalized;
            m_Move = v * m_CamForward + h * m_Cam.right;
        }
        else
        {
            // 메인 카메라가 없는 경우 world-relative 방향을 사용
            m_Move = v * Vector3.forward + h * Vector3.right; // v * (0,0,1) + h * (1,0,0)
        }

        m_Move = m_Move.normalized;

////
#if !MOBILE_INPUT // 모바일 입력이 비활성화된 상태인 경우에만 아래 코드 컴파일
        // walk speed multiplier
        if (Input.GetKey(KeyCode.LeftShift)) m_Move *= 0.5f;
#endif
/////
        if (crouch)
        {
            m_Move *= 1.0f;
        }
        else
        {
            m_Move *= 2.0f;
        }

        // 이동 벡터, 웅크리고 있는 상태인지, 점프중인 상태인지에 대한 정보를 전달
        m_Character.Move(m_Move, crouch, m_Jump);

        // 디버깅 관련일 듯
        inputText.text = "Input";
        inputText.text += "\n";
        inputText.text += " velocity : " + m_Move.magnitude.ToString();
        inputText.text += "\n";
        inputText.text += " direction : " + m_Move.normalized;
        inputText.text += "\n";
        inputText.text += crouch ? " crouch : true" : " crouch : false";
        inputText.text += "\n";
        inputText.text += m_Jump ? " jump : true" : " jump : false";

        // 점프값 초기화
        m_Jump = false;
    }
}