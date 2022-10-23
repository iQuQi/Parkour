using UnityEngine;

//[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(CapsuleCollider))]
[RequireComponent(typeof(Animator))]
[RequireComponent(typeof(MotionMatcher))]
public class PlayerController : MonoBehaviour
{

    // 각종 값 초기화
    // [SerializeField]: 이 필드는 비공개임에도 일련화 됨
    [SerializeField] float m_MovingTurnSpeed = 360;
    [SerializeField] float m_StationaryTurnSpeed = 180;
    [SerializeField] float m_JumpPower = 12f; // 쓰이는 곳 없음
    [Range(1f, 4f)] [SerializeField] float m_GravityMultiplier = 2f; // 쓰이는 곳 없음
    [SerializeField] float m_RunCycleLegOffset = 0.2f; // 샘플 asset의 특성에 따라 다른 asset과 함께 작업하려면 수정해야 함 & 쓰이는 곳 없음
    [SerializeField] float m_MoveSpeedMultiplier = 1f;
    [SerializeField] float m_AnimSpeedMultiplier = 1f; // 쓰이는 곳 없음
    [SerializeField] float m_GroundCheckDistance = 0.1f;
    [SerializeField] AnimationClip[] animationClips; // 쓰이는 곳 없음


    //Start에서 초기화
    private MotionMatcher motionMatcher; // 깃헙에 정의된 클래스
    private string matchedMotionName = "HumanoidIdle";

       //Rigidbody m_Rigidbody;
    Animator m_Animator;
    bool m_IsGrounded;
    float m_OrigGroundCheckDistance;

    float m_CapsuleHeight;
    Vector3 m_CapsuleCenter;
    CapsuleCollider m_Capsule;

    // Move에서 초기화
    float m_TurnAmount;
    float m_ForwardAmount;
    bool m_Crouching;

    // 나머지
    const float k_Half = 0.5f; // 쓰이는 곳 없음
    Vector3 m_GroundNormal; // 값 초기화가 안된 상태로 아래에서 쓰임
    float fps = 30.0f;
    


    void Start()
    {
        // 게임 오브젝트를 가져오기
        m_Animator = GetComponent<Animator>(); // Animator 컴포넌트는 씬에서 게임 오브젝트에 애니메이션을 지정하는 데 사용. 어떤 애니메이션 클립을 사용할 것인지, 언제 어떻게 블렌드 및 전환할 것인지를 정의.
            //m_Rigidbody = GetComponent<Rigidbody>(); // Rigidbody 는 GameObject 가 물리 제어로 동작하게 함
        m_Capsule = GetComponent<CapsuleCollider>(); // 충돌 감지를 위한 컴포넌트
        motionMatcher = GetComponent<MotionMatcher>();

        // 각종 변수 초기화
        m_CapsuleHeight = m_Capsule.height;
        m_CapsuleCenter = m_Capsule.center;
            //m_Rigidbody.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationY | RigidbodyConstraints.FreezeRotationZ;
        m_OrigGroundCheckDistance = m_GroundCheckDistance;
        m_IsGrounded = true;
        m_Animator.applyRootMotion = true;
    }

    // 상위 클래스에서 호출하는 메인 함수
    public void Move(Vector3 move, bool crouch, bool jump)
    {
        // world-relative 'move' 입력벡터를
        // 원하는 방향으로 향하기 위해 필요한 
        // local-relative 회전량과 앞으로 향하는 양으로 변환

        if (move.magnitude > 1f) move.Normalize(); // 크기 1로 변환

        move = transform.InverseTransformDirection(move); // 세계 공간에서 로컬 공간으로 변환. Transform.TransformDirection의 반대
        move = Vector3.ProjectOnPlane(move, m_GroundNormal); // 평면에 수직인 법선으로 정의된 평면에 벡터를 투영

        m_TurnAmount = Mathf.Atan2(move.x, move.z); // 로컬 상대적인 회전량 계산 (Atan2: 두점사이의 절대각도를 측정하는 함수)
        m_ForwardAmount = move.z; // 로컬 상대적인 앞방향으로의 움직임량 계산

        // 회전을 더 빨리 할 수 있게 해주는 함수 호출
        ApplyExtraTurnRotation();

        m_Crouching = crouch;

        // 입력 및 기타 상태 매개 변수를 Animator로 보내기
        UpdateAnimator(move);
    }

    void UpdateAnimator(Vector3 move)
    {
        // 애니메이터 매개 변수 업데이트 ..? 아래 코드들 미완성인가
            // m_Animator.SetFloat("Forward", m_ForwardAmount, 0.1f, Time.deltaTime);
            // m_ForwardAmount
            // m_TurnAmount
            // m_Crouching

        float normalizedTime = m_Animator.GetCurrentAnimatorStateInfo(0).normalizedTime;
        normalizedTime = normalizedTime - Mathf.FloorToInt(normalizedTime);

        PlayerInput playerInput = new PlayerInput(); // 유니티 제공 클래스

        // playerInput의 속도, 방향, 각속도 초기화
        if (m_ForwardAmount >= 1)
        {
            playerInput.velocity = move.magnitude;
            playerInput.direction = move.normalized;
            playerInput.angularVelocity = 0.0f;
        }
        else
        {
            playerInput.velocity = 0.0f;
            playerInput.direction = Vector3.zero;
            playerInput.angularVelocity = Mathf.Lerp(m_StationaryTurnSpeed, m_MovingTurnSpeed, m_ForwardAmount) * m_TurnAmount;
        }

        // playerInput의 가속도, 브레이크, 쭈그림 초기화
        playerInput.acceleration = 0.0f;
        playerInput.brake = 0.0f;
        playerInput.crouch = m_Crouching;


        // 모션 매칭 기법으로 적합한 모션 찾기.  매개변수로 플레이어 입력, 현재 모션이름, 정규화한 시간을 전달
        string matchedMotionName = motionMatcher.AcquireMatchedMotion(playerInput, this.matchedMotionName, normalizedTime);

        // 받아온 모션이 현재 모션과 다르다면 업데이트 진행
        if (this.matchedMotionName != matchedMotionName)
        {
            // 모션 관련 값 업데이트
            this.matchedMotionName = matchedMotionName;
                //float bestMotionFrameIndex = motionMatcher.bestMotionFrameIndex;
            float bestMotionFrameIndex = 0;
            float bestMotionTime = bestMotionFrameIndex / (m_Animator.GetCurrentAnimatorStateInfo(0).length * fps);
                //m_Animator.Play(matchedMotionName, 0, bestMotionTime);

            // CrossFade 블렌딩 함수를 통해 부드럽게 애니메이션의 변화가 이뤄질수 있게 해줌.
            m_Animator.CrossFade(matchedMotionName, 0.1f); 

            // 디버깅 코드
            Debug.Log("Play matchedMotionName " + matchedMotionName);
        }
    }


    void ApplyExtraTurnRotation()
    {
        // 캐릭터가 더 빨리 회전할 수 있도록 도와줌 (애니메이션에서 루트 회전도)
        float turnSpeed = Mathf.Lerp(m_StationaryTurnSpeed, m_MovingTurnSpeed, m_ForwardAmount); // Lerp: 숫자간의 선형 보간, 마지막 인자는 t값
        
        // Time.deltaTime: 마지막 프레임에서 현재 프레임까지의 간격(초)
        transform.Rotate(0, m_TurnAmount * turnSpeed * Time.deltaTime, 0);
    }

    // 이 함수 사용되는 곳이 없음 -> 코드 미완성?
    public void OnAnimatorMove()
    {
        // 기본 루트 모션을 재정의하기 위해 필요한 함수 => 적용되기 전에 위치 속도를 수정 가능
        if (m_IsGrounded && Time.deltaTime > 0)
        {
            Vector3 v = (m_Animator.deltaPosition * m_MoveSpeedMultiplier) / Time.deltaTime;

            // 현재 속도의 기존 y 부분을 보존
                // v.y = m_Rigidbody.velocity.y;
                // m_Rigidbody.velocity = v;
        }
    }
}